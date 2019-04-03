/**
 * "drive_types.h"
 * struct and enum definitions for the drive program
 */


#ifndef DRIVE_TYPES_H
#define DRIVE_TYPES_H

#define MAX_WAYPOINTS 24

#define W_FRONT_LEFT	0
#define W_FRONT_RIGHT	1
#define W_REAR_LEFT	2
#define W_REAR_RIGHT	3
/**
 * @brief      4 driving modes for the wheels
 */
typedef enum steer_mode_t{
	NORMAL_STEERING,	///< Uses onlty front 2 wheels to steer
	AW_STEERING,		///< Uses all 4 wheels with the rear coutner steering relative to the front
	PARALLEL_STEERING,	///< Uses all 4 wheels to steer while keep all of them parallel
	SPIN			///< Uses all 4 wheels to spin rover around centroid
} steer_mode_t;

/**
 * @brief single waypoint definition used by task_t
 */
typedef struct waypoint_t{
	float x;
	float y;
	float heading; // heading in rad
} waypoint_t;

/**
 * @brief      basically a series of waypoints
 */
typedef struct task_t {
	steer_mode_t steer_mode;
	int num_waypoints;
	waypoint_t waypoints[MAX_WAYPOINTS];
} task_t;


/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
	arm_state_t arm_state;		///< see arm_state_t declaration
	steer_mode_t steer_mode;	///< FULL or FRONT Only steering
	float forward_vel;		///< forward velocity of vehicle
	float steering_angle;		///< gamma, positive left.
	float icr; 			///<ICR- instantaneous centre of rotation for steering, distance from center of rover (perpendicular)
	float out_steering_angle;	///< outer wheel steering angle
	float in_steering_angle;	///< inner wheel steering angle
	float out_steering_input;	///< outer steering angle setpoint
	float in_steering_input;	///< inner steering anngle setpoint
	float pos_x;			///< global rover x position setpoint
	float pos_x_dot;		///< global velocity in x direction
	float pos_y;			///< global rover y position setpoint
	float pos_y_dot;		///< global velocity in y direction
	float theta;			///< global rover heading (rad)
	float theta_dot;		///< rate at which gamma setpoint updates (rad/s)
	float beta;			///< global final rover heading pose in goto pose mode (rad)
	int current_waypoint;		///< only used in waypoint follow mode, read by feedback controller, set by setpoint_manager
}setpoint_t;


/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
	/** @name wheel position and velocties*/
	///@{
	float steer_angle[4];		///< wheel steering angle from straight forward radians
	float wheel_angle[4];		///< wheel rotation position from start in radians
	float wheel_speed[4];		///< wheel rotation speed in forward direction in rad/s
	float forward_velocity;
	float last_avg_wheel_angle;
	///@}

	/** @name global position and velocities **/
	///@{
	float pos_x;
	float pos_x_dot;
	float pos_y;
	float pos_y_dot;
	float heading;
	float heading_dot;
	///@}

	/** @name distance and heading to waypoint, ONLY IN WAYPOINT MODE **/
	///@{
	float rho;		///< distance to waypoint (m)
	float alpha;		///< heading to waypoint (rad) positive is waypoint is to the left of current car direction
	float wp_heading;	///< heading to wp from pos.
	///@}

	//additional states
	float vBatt;		///< battery voltage
	float camera_pitch;	///< radians from level, positive up
	float mot_drive[];	///< u compensated for battery voltage
} core_state_t;


#endif