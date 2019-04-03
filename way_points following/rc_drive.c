/**
 * @file rc_drive.c
 *
 * This is meant to be a program to control the edurover (available at
 * https://github.com/StrawsonDesign/EduRover). Code structure built from
 * rc_balance.c in Examples of Robotics Cape Library Full Credit to James
 * Strawson for the Robotics Cape Library and assistance to create this code.
 * Last edited by Dominique Meyer on 06/08/2018 for CSE291D@UCSD
 */

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <getopt.h>
#include <signal.h>
#include <math.h> // for M_PI
#include <roboticscape.h> // includes ALL robotics cape subsystems
#include <rc/mavlink_udp.h>
#include <rc/mavlink_udp_helpers.h>

#include "drive_types.h"
#include "rc_drive_defs.h" // includes all parameters defining the edurover





/////////////////////////// FUNCTION DECLARATIONS //////////////////////////////

void controller();			///< mpu ISR feedback controller
void* setpoint_manager(void* ptr);	///< setpoint manager background thread
void* battery_checker(void* ptr);	///< background  batterythread
void* printf_loop(void* ptr);		///< background thread

int zero_out_controller();
int disarm_controller();
int arm_controller();
int wait_for_starting_condition();
void on_pause_press();
void callback_func_any();


/////////////////////////// GLOBAL VARIABLES ////////////////////////////////////

core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D1;		///< velocity PID controller
rc_mpu_data_t mpu_data;
task_t task;

const char* dest_ip;
uint8_t my_sys_id;
uint16_t port;
FILE *fp;


/////////////////////////////// MAIN LOOP ////////////////////////////////////


/**
 * @brief Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	pthread_t setpoint_thread = 0;
	pthread_t battery_thread = 0;
	pthread_t printf_thread = 0;

	// parse arguments
	opterr = 0;

	// load in task waypoints
	#include "hw2_task.h"

///////////////////// SIGNAL HANDLER/ PID FILE /////////////////////////////////

	// make sure another instance isn't running
	// return value -3 means a root process is running and we need more
	// privileges to stop it.
	if(rc_kill_existing_process(2.0)==-3) return -1;
	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	// make PID file to indicate project is running
	rc_make_pid_file();

//////////// INIT BASIC HARDWARE, MOTORS, BUTTONS, LEDS ////////////////////////

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
			RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
			RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,NULL);

	// initialize enocders
	if(rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return -1;
	}
	if(rc_encoder_pru_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize pru encoder\n");
		return -1;
	}

	// initialize motors
	if(rc_motor_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(0); // start with motors not in standby

	// initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
		return -1;
	}


	// start with RED lef on, turn off after init
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}


////////////////////////////////    MPU/DMP    /////////////////////////////////

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	//mpu_config.enable_magnetometer = 1;

	//run calibration routine for gyro
	rc_mpu_calibrate_gyro_routine(mpu_config);

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;
	D1 = rc_filter_empty();

	// set up D1 Velocity Controller
	if(rc_filter_pid(&D1, D1_KP, D1_KI, D1_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_drive, failed to make velocity PID controller\n");
		return -1;
	}
	rc_filter_enable_saturation(&D1, -1.0, 1.0);

//////////////////////////////// START THREADS /////////////////////////////////

	// start a thread to slowly sample battery
	if(rc_pthread_create(&battery_thread, battery_checker, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}
	// wait for the battery thread to make the first read
	while(cstate.vBatt==0 && rc_get_state()!=EXITING) rc_usleep(1000);
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		if(rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start printf thread\n");
			return -1;
		}
	}
	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}
	// start balance stack to control setpoints
	if(rc_pthread_create(&setpoint_thread, setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start setpoint controller thread\n");
		return -1;
	}
	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&controller);

	// start Mavlink thread

	// set default options before checking options
	dest_ip = MAVLINK_HOST_IP;
	my_sys_id = DEFAULT_SYS_ID;
	port = RC_MAV_DEFAULT_UDP_PORT;

	if(rc_mav_init(my_sys_id, dest_ip, port,RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US)==0){
		rc_mav_set_callback_all(callback_func_any);
		printf("Mavlink initialized\n");
	}
	else{
		return -1;
	}

	// starting file log and assigning pointer.
	fp = fopen("log.txt","w");




//////////////// TELL USER SETUP IS DONE AND TWIDDLE THUMBS ////////////////////

	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	printf("\nHit the MODE button to begin path following\n");
	rc_set_state(RUNNING);

	// chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_usleep(500000);
	}


////////////////       JOIN THREADS AND CLEAN UP           ////////////////////
	// join threads
	rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
	rc_pthread_timed_join(battery_thread, NULL, 1.5);
	rc_pthread_timed_join(printf_thread, NULL, 1.5);

	// cleanup
	rc_filter_free(&D1);
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_encoder_pru_cleanup();
	rc_button_cleanup();		// stop button handlers
	rc_remove_pid_file();		// remove pid file LAST
	return 0;
}


int ar;

////////////////////////// Setpoint Manager ////////////////////////////////////
/**
 * @brief Adjusts the controller setpoint based on user
 * input of waypoint files.
 *
 * @param      ptr   The pointer
 *
 * @return    NULL
 */
void* setpoint_manager(__attribute__ ((unused)) void* ptr)
{

	// make sure waypoint counter starts at 0
	// wait for mpu to settle
	disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);

	while(rc_get_state()!=EXITING){

		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING) continue;
		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(setpoint.arm_state == DISARMED){
			if(wait_for_starting_condition()==0){
				zero_out_controller();
				arm_controller();
			}
			else continue;
		}

		// if no waypoint reached, set next setpoints
		if(cstate.rho >= WAYPOINT_ACCEPTANCE_RADIUS){
			continue;
		}

		//if reached current waypoint goal, set next waypoint as goal
		if(cstate.rho <= WAYPOINT_ACCEPTANCE_RADIUS){
			if(setpoint.current_waypoint >= NUM_WAYPOINTS){
				fprintf(stderr, "\nRover has Reached Final Waypoint %d \n",setpoint.current_waypoint);
				disarm_controller();
				rc_set_state(PAUSED);
			}
			else{
				fprintf(stderr, "\nRover has Reached Waypoint %d \n",setpoint.current_waypoint);
				setpoint.current_waypoint++;
				setpoint.pos_x = task.waypoints[setpoint.current_waypoint].x;
				setpoint.pos_y = task.waypoints[setpoint.current_waypoint].y;
				setpoint.beta  = task.waypoints[setpoint.current_waypoint].heading;
				rc_usleep(50000000);
				continue;
			}
		}

		// clear out input of old data before waiting for new data
		rc_usleep(1000000/SETPOINT_MANAGER_HZ);

	}
	// if state becomes EXITING the above loop exists and we disarm here
	disarm_controller();
	return NULL;
}

//////////////////////////////// MAIN CONTROLLER ////////////////////////////////////////

/**
 * discrete-time motion controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
void controller()
{
	int i;
	float current_avg_wheel_angle;
	float current_avg_wheel_speed;

	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	// collect encoder positions, right wheel is reversed
	cstate.wheel_angle[1] = (rc_encoder_eqep_read(ENCODER_CHANNEL_FL) * 2.0 * M_PI) \
		/(ENCODER_POLARITY_FL * GEARBOX * ENCODER_RES);
	cstate.wheel_angle[2] = (rc_encoder_eqep_read(ENCODER_CHANNEL_FR) * 2.0 * M_PI) \
		/(ENCODER_POLARITY_FR * GEARBOX * ENCODER_RES);

	current_avg_wheel_angle = (cstate.wheel_angle[1] + cstate.wheel_angle[2])/2.0;
	current_avg_wheel_speed = (current_avg_wheel_angle - cstate.last_avg_wheel_angle)/DT;

	cstate.forward_velocity = current_avg_wheel_speed*WHEEL_RADIUS_M;
	//cstate.heading = mpu_data.compass_heading;
	//cstate.heading_dot = mpu_data.gyro[2]*DEG_TO_RAD-(M_PI/2);

	//cstate.pos_x = cstate.pos_x + cos(cstate.heading)*cstate.forward_velocity*DT;
	//cstate.pos_y = cstate.pos_y + sin(cstate.heading)*cstate.forward_velocity*DT;

	// save last angle
	cstate.last_avg_wheel_angle = current_avg_wheel_angle;

	// distance to next waypoint
	cstate.rho = sqrt (pow((task.waypoints[setpoint.current_waypoint].x - cstate.pos_x),2) + pow((task.waypoints[setpoint.current_waypoint].y - cstate.pos_y),2));
	setpoint.forward_vel = KRHO*cstate.rho;

	if (setpoint.forward_vel < 0.3){
		setpoint.forward_vel = 0.3;
	}
	else if (setpoint.forward_vel > 0.5){
		setpoint.forward_vel = 0.5;
	}



	// angle to waypoint from current pos
	cstate.wp_heading = atan2((task.waypoints[setpoint.current_waypoint].x - cstate.pos_x),(task.waypoints[setpoint.current_waypoint].y - cstate.pos_y));

	cstate.wp_heading = cstate.wp_heading + M_PI;

	if (cstate.wp_heading >= M_PI){
		cstate.wp_heading = cstate.wp_heading - M_PI;
	}
	else if(cstate.wp_heading < M_PI){
		cstate.wp_heading = cstate.wp_heading + M_PI;
	}

	// if (cstate.wp_heading <0.0){
	// 	cstate.wp_heading += M_PI*2;
	// }
	// cstate.alpha = cstate.wp_heading - cstate.heading- M_PI/4;
	// if(cstate.alpha > M_PI){
	// 	cstate.alpha = (cstate.wp_heading - cstate.heading- M_PI/4) - M_PI*2;
	// }

	// if((cstate.wp_heading - cstate.heading) > M_PI*2){
	// 	cstate.alpha = -(cstate.wp_heading + cstate.heading-M_PI/4);
	// }
	// else{
	// 	cstate.alpha = cstate.wp_heading - cstate.heading - M_PI/4;
	// }
	cstate.alpha = cstate.wp_heading - cstate.heading;

	if(cstate.alpha > M_PI){
		cstate.alpha = cstate.alpha - M_PI*2;
	}
	else if(cstate.alpha < - M_PI){
		cstate.alpha = cstate.alpha + M_PI*2;
	}

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_motor_standby(0);
		for(i=1;i<5;i++){
			rc_motor_set(i,0.0);
		}

	return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}

	/*************************************************************
	*steering angle derivation
	***************************************************************/

	// calculate steering angle using Alpha and Beta with some tuning variables

	setpoint.steering_angle = (KA*cstate.alpha + KB*setpoint.beta)/2;

	if(setpoint.steering_angle >= AWD_MAX_STEERING_ANGLE){
		setpoint.steering_angle = AWD_MAX_STEERING_ANGLE;
	}
	else if(setpoint.steering_angle <= -AWD_MAX_STEERING_ANGLE){
		setpoint.steering_angle = -AWD_MAX_STEERING_ANGLE;
	}

	/*************************************************************
	*Ackerman Steering Calculations and Output
	***************************************************************/

	setpoint.icr = (WHEEL_FORWARD_BASELINE/2)/tan(setpoint.steering_angle);
	setpoint.out_steering_angle = atan((WHEEL_FORWARD_BASELINE/2)/(setpoint.icr + WHEEL_SIDE_BASELINE/2));
	setpoint.in_steering_angle = atan((WHEEL_FORWARD_BASELINE/2)/(setpoint.icr - WHEEL_SIDE_BASELINE/2));

	setpoint.out_steering_input = (setpoint.out_steering_angle*WHEEL_PI_INPUT / (M_PI/2));
	setpoint.in_steering_input = (setpoint.in_steering_angle*WHEEL_PI_INPUT / (M_PI/2));



	/**********************************************************
	* velocity PID controller D1
	***********************************************************/
	float new_motor_power; // net power to motors, from -1 to 1
	new_motor_power = rc_filter_march(&D1,setpoint.forward_vel- cstate.forward_velocity);
	//fprintf(stderr, "Set forward %f \n",setpoint.forward_vel);
	//fprintf(stderr, "Speed %f \n",new_motor_power);



	/**********************************************************
	* Send signal to motors and servos
	***********************************************************/


	rc_motor_set(1,new_motor_power*MOTOR_POLARITY_FL);
	cstate.mot_drive[0] = new_motor_power*MOTOR_POLARITY_FL;

	rc_motor_set(2,new_motor_power*MOTOR_POLARITY_FR);
	cstate.mot_drive[1] = new_motor_power*MOTOR_POLARITY_FR;

	rc_motor_set(3,new_motor_power*MOTOR_POLARITY_RR);
	cstate.mot_drive[2] = new_motor_power*MOTOR_POLARITY_RR;

	rc_motor_set(4,new_motor_power*MOTOR_POLARITY_RL);
	cstate.mot_drive[3] = new_motor_power*MOTOR_POLARITY_RL;



	if(setpoint.steering_angle < 0){
		rc_servo_send_pulse_normalized(1, (SERVO_CENTER_FL - WHEEL_STRAIGHT_INPUT + setpoint.in_steering_input));
		cstate.steer_angle[1] = setpoint.in_steering_angle;
		rc_servo_send_pulse_normalized(2, (SERVO_CENTER_FR + WHEEL_STRAIGHT_INPUT + setpoint.out_steering_input));
		cstate.steer_angle[2] = setpoint.out_steering_angle;
		rc_servo_send_pulse_normalized(3, (SERVO_CENTER_RR - WHEEL_STRAIGHT_INPUT - setpoint.out_steering_input));
		cstate.steer_angle[3] = setpoint.out_steering_angle;
		rc_servo_send_pulse_normalized(4, (SERVO_CENTER_RL + WHEEL_STRAIGHT_INPUT - setpoint.in_steering_input));
		cstate.steer_angle[4] = setpoint.in_steering_angle;
	}
	else if(setpoint.steering_angle > 0){
		rc_servo_send_pulse_normalized(1, (SERVO_CENTER_FL - WHEEL_STRAIGHT_INPUT + setpoint.out_steering_input));
		cstate.steer_angle[1] = setpoint.in_steering_angle;
		rc_servo_send_pulse_normalized(2, (SERVO_CENTER_FR + WHEEL_STRAIGHT_INPUT + setpoint.in_steering_input));
		cstate.steer_angle[2] = setpoint.out_steering_angle;
		rc_servo_send_pulse_normalized(3, (SERVO_CENTER_RR - WHEEL_STRAIGHT_INPUT - setpoint.in_steering_input));
		cstate.steer_angle[3] = setpoint.out_steering_angle;
		rc_servo_send_pulse_normalized(4, (SERVO_CENTER_RL + WHEEL_STRAIGHT_INPUT - setpoint.out_steering_input));
		cstate.steer_angle[4] = setpoint.in_steering_angle;
	}
	else if(setpoint.steering_angle == 0){
		rc_servo_send_pulse_normalized(1, (SERVO_CENTER_FL - WHEEL_STRAIGHT_INPUT));
		cstate.steer_angle[1] = 0;
		rc_servo_send_pulse_normalized(2, (SERVO_CENTER_FR + WHEEL_STRAIGHT_INPUT));
		cstate.steer_angle[2] = 0;
		rc_servo_send_pulse_normalized(3, (SERVO_CENTER_RR - WHEEL_STRAIGHT_INPUT));
		cstate.steer_angle[3] = 0;
		rc_servo_send_pulse_normalized(4, (SERVO_CENTER_RL + WHEEL_STRAIGHT_INPUT));
		cstate.steer_angle[4] = 0;

	}




	return;
}



/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
int zero_out_controller()
{
	int i;
	rc_filter_reset(&D1);
	// zero out setpoint
	setpoint.pos_x =	0.0;
	setpoint.pos_x_dot =	0.0;
	setpoint.pos_y =	0.0;
	setpoint.pos_y_dot =	0.0;

	setpoint.current_waypoint = 0;

	// zero out state
	cstate.pos_x =		0.0;
	cstate.pos_x_dot =	0.0;
	cstate.pos_y =		0.0;
	cstate.pos_y_dot =	0.0;
	cstate.rho =		0.0;
	cstate.alpha =		0.0;

	for(i=0;i<4;i++){
		cstate.steer_angle[i]=0.0;
		cstate.wheel_angle[i]=0.0;
		cstate.wheel_speed[i]=0.0;
	}

	cstate.last_avg_wheel_angle = 0.0;

	rc_motor_standby(0);
	for(i=1;i<5;i++){
		rc_motor_set(i,0.0);
	}
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
int disarm_controller()
{
	rc_motor_standby(1);

	setpoint.arm_state = DISARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
int arm_controller()
{
	rc_encoder_eqep_write(1,0);
	rc_encoder_eqep_write(2,0);
	rc_encoder_eqep_write(3,0);
//	rc_encoder_pru_write(0);
	rc_motor_standby(0);
	rc_servo_init();
	rc_servo_power_rail_en(1);
	setpoint.arm_state = ARMED;
	return 0;
}

/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
int wait_for_starting_condition()
{
	// wait for MiP to be tipped back or forward first
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		rc_usleep(100000);
		// if within range, start counting
		if(rc_button_get_state(RC_BTN_PIN_MODE)==RC_BTN_STATE_RELEASED) continue;
		else return 0;
	}
	return -1;
}

/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
void* battery_checker(__attribute__ ((unused)) void* ptr){
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_batt();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/**
 * Call back function called by rc_mav library when Mavlink Packet is received.
 * function writes to cstate all information received from mavlink.
 */

void callback_func_any(){
	int i;
	// init variables to store data locally in function
	mavlink_att_pos_mocap_t pos_data;	// struct to contain mavlink packet data
	double q[4];				// will contain angles
	double tb[4];				// will contain angles

	// let mavlink function write to struct given by pointer
	rc_mav_get_att_pos_mocap(&pos_data);

	//update rover position
	cstate.pos_x = pos_data.x/1000;
	cstate.pos_y = pos_data.y/1000;

	for(i=0; i<4 ; i++){
		q[i] = pos_data.q[i];
	}

	//get heading
	rc_quaternion_to_tb_array(q, tb);

	cstate.heading = -tb[0];

	cstate.heading = cstate.heading + M_PI ;

	if(cstate.heading >= M_PI/2){
		cstate.heading = cstate.heading - M_PI/2;
	}
	else if(cstate.heading < M_PI/2){
		cstate.heading = cstate.heading + M_PI *3/2;
	}


	return;
}



/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
void* printf_loop(__attribute__ ((unused)) void* ptr){
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("s.waypnt |");
			printf("  c.rho  |");
			printf("c.for vel|");
			printf(" c.pos x |");
			printf(" s.pos x |");
			printf(" c.pos y |");
			printf(" s.pos y |");
			printf(" s.vel   |");
			printf("c.heading|");
			printf("s.osteeri|");
			printf("s.isteeri|");
			printf("c. alpha |");
			printf("c. wp_hea|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			printf("\r");
			printf("%7d  |", setpoint.current_waypoint);
			printf("%7.3f  |", cstate.rho);
			printf("%7.3f  |", cstate.forward_velocity);
			printf("%7.3f  |", cstate.pos_x);
			printf("%7.3f  |", setpoint.pos_x);
			printf("%7.3f  |", cstate.pos_y);
			printf("%7.3f  |", setpoint.pos_y);
			printf("%7.3f  |", setpoint.forward_vel);
			printf("%7.3f  |", cstate.heading);
			printf("%7.3f  |", setpoint.out_steering_input);
			printf("%7.3f  |", setpoint.in_steering_input);
			printf("%7.3f  |", cstate.alpha);
			printf("%7.3f  |", cstate.wp_heading);
			printf("  ");


			//if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			//else printf("DISARMED |");
			fflush(stdout);

			fprintf(fp, "%7.3f,%7.3f,%7.3f\n",cstate.pos_x,cstate.pos_y, cstate.heading);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
void on_pause_press()
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		rc_led_set(RC_LED_RED,1);
		rc_led_set(RC_LED_GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		disarm_controller();
		rc_led_set(RC_LED_GREEN,1);
		rc_led_set(RC_LED_RED,0);
		break;
	default:
		break;
	}

	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_led_blink(RC_LED_RED,5,1);
	rc_set_state(EXITING);
	return;
}

