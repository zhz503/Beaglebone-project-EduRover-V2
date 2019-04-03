/**
 * rc_drive_defs.h
 *
 * Contains the physical parameters and constraints for the edurover used in rc_drive.c
 */

#ifndef RC_DRIVE_CONFIG
#define RC_DRIVE_CONFIG


#define SAMPLE_RATE_HZ		100			// main filter and control loop speed
#define DT			(1.0/SAMPLE_RATE_HZ)	// 1/sample_rate

// Structural properties of ardurover
#define CAPE_MOUNT_ANGLE	0.49	// increase if mip tends to roll forward
#define GEARBOX			35.577	// gear ratio between motor output shaft and
#define ENCODER_RES		60	//encoder resolution per rotation (motor output shaft)
#define WHEEL_RADIUS_M		0.0425	//wheel radius in meters
#define WHEEL_SIDE_BASELINE	0.12	//wheel sideways baseline separation between axis of rotation in meters
#define WHEEL_FORWARD_BASELINE	0.17	//wheel forward baseline separation between axis of rotation in meters

//steering parameters
#define WHEEL_ANGLE_STRAIGHT	37.5	// angle in deg for wheels to be straight from 0 position (pos for lef two, neg for right two)
#define WHEEL_STRAIGHT_INPUT	0.6	// normalized input to straighten wheels over angle of 37.5deg
#define WHEEL_PI_INPUT		1.45	//scaled input to servo for 90deg rotation
#define AWD_MAX_STEERING_IN	0.3	//max steering input when in AWD Steering mode
#define AWD_MAX_STEERING_ANGLE	0.26	// max steering angle in rad.

// electrical hookups
#define MOTOR_CHANNEL_FL	1
#define MOTOR_CHANNEL_FR	2
#define MOTOR_CHANNEL_RL	4
#define MOTOR_CHANNEL_RR	3
#define MOTOR_POLARITY_FL	-1
#define MOTOR_POLARITY_FR	1
#define MOTOR_POLARITY_RR	1
#define MOTOR_POLARITY_RL	-1
#define ENCODER_CHANNEL_FL	1
#define ENCODER_CHANNEL_FR	2
#define ENCODER_CHANNEL_RL	4
#define ENCODER_CHANNEL_RR	3
#define ENCODER_POLARITY_FL	1
#define ENCODER_POLARITY_FR	-1
#define ENCODER_POLARITY_RL	1
#define ENCODER_POLARITY_RR	-1

#define SERVO_CHANNEL_FL	1
#define SERVO_CHANNEL_FR	2
#define SERVO_CHANNEL_RR	3
#define SERVO_CHANNEL_RL	4

#define SERVO_CHANNEL_GIMBAL	5

#define V_NOMINAL		7.4

// Servo Calibration Settings

#define SERVO_MAX_FL	0.8
#define SERVO_MIN_FL	-1.1
#define SERVO_VER_FL	-0.8
#define SERVO_HOR_FL	0.65
#define SERVO_CENTER_FL	-0.2


#define SERVO_CENTER_FR	-0.12

#define SERVO_CENTER_RR	-0.05

#define SERVO_CENTER_RL	-0.05



// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	100
#define PRINTF_HZ		10

// Navigation Parameters
#define WAYPOINT_ACCEPTANCE_RADIUS 0.3
#define KRHO 0.5 // tuning constant for velocity convergence
#define KA 1 //tuning constant for angle convergence
#define KB 0 //tuning constant for angle convergence

// Servo range parameters
#define DIST_FROM_MID_TO_FORWARD 0.55
#define DIST_FROM_MID_TO_SIDEWAYS 0.90

// forward velocity PID constants
#define D1_KP 1.5
#define D1_KI 0.00
#define D1_KD 0.0

// Mavlink Configs

#define MAVLINK_HOST_IP "192.168.1.2"
#define DEFAULT_SYS_ID 1


#endif	// endif RC_BALANCE_CONFIG
