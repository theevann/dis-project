/*****************************************************************************/
/* File:         follower.c                                                  */
/* Version:      3.0                                                         */
/* Date:         02-Nov-05 -- 06-Oct-2015                                    */
/* Description:  Formation movement with E-Pucks                             */
/*                                                                           */
/* Author: 	 22-Oct-04 by nikolaus.correll@epfl.ch                       */
/* improved by Ali Marjovi 20-Oct-2014 and 06-Oct 2015			     */
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
#include <webots/accelerometer.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "odometry.h"

#define NB_SENSORS           8

/* Formation flocking parameters */
#define D                    0.20      // Distance between robots
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
#define DELTA_T			0.064	// Timestep (seconds)
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define FLOCK_SIZE	  4	  // Size of flock
/*Webots 2018b*/
#define ROBOTS_N 4



int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};	
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;		
WbDeviceTag receiver;	
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
	


int robot_id;                       // Unique robot ID
int flocking_group, robot_flock_id;
float goal_range   = 0.0;
float goal_bearing = 0.0;

float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;

//position_t relative_pos[FLOCK_SIZE];	// relative X, Z, Theta of all robots
position_t my_position;     		// X, Z, Theta of the current robot
static position_t flock_rpos[ROBOTS_N];

static void reset(int ts) {
	wb_robot_init();

	dev_gps = wb_robot_get_device("gps");
    wb_gps_enable(dev_gps, 1000);

    dev_acc = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(dev_acc, ts);

    dev_left_encoder = wb_robot_get_device("left wheel sensor");
    dev_right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(dev_left_encoder, ts);
    wb_position_sensor_enable(dev_right_encoder, ts);

    dev_left_motor = wb_robot_get_device("left wheel motor");
    dev_right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(dev_left_motor, INFINITY);
    wb_motor_set_position(dev_right_motor, INFINITY);
    wb_motor_set_velocity(dev_left_motor, 0.0);
    wb_motor_set_velocity(dev_right_motor, 0.0);
    receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");

	int i;

	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);      // the device name is specified in the world file
		s[2]++;                         // increases the device number
	}
	char* robot_name; robot_name=(char*) wb_robot_get_name(); 

	sscanf(robot_name,"epuck%d",&robot_id);    // read robot id from the robot's name
	printf("Reset: robot %d\n",robot_id);

	switch (robot_id)
	{
	case 1:
		goal_range = sqrt(2 * pow(0.15, 2));
		goal_bearing = atan(1);
		break;
	case 2:
		goal_range = 0.3;
		goal_bearing = 0;
		break;
	case 3:
		goal_range = sqrt(2 * pow(0.15, 2));
		goal_bearing = -atan(1);

		break;
	default:
		break;
	}
	leader_range = goal_range;
	leader_bearing = goal_bearing;
	printf("the value of the goal range and bearing for the robot %d is %f and %f and %f \n", robot_id, leader_range, leader_bearing, leader_orientation);
	robot_flock_id = robot_id % FLOCK_SIZE;
}

void update_leader_measurement(float new_leader_range, float new_leader_bearing) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
	//leader_orientation = new_leader_orientation;
}

void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr) {
	// Define constants
	float Ku = 2.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	//float theta = leader_orientation;
	x += goal_range * cosf(- M_PI + goal_bearing + flock_rpos[0].heading);
	y += goal_range * sinf(- M_PI + goal_bearing + flock_rpos[0].heading);
	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float w = Kw * range * sinf(bearing) + Kb * flock_rpos[0].heading;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)
	printf("the value of u is %f and the value of w is %f\n", u, w);
	// Convert to wheel speeds!
	*msl = (int)((u - AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}



/*void process_received_ping_messages(){
    const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[1];
		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		relative_pos[other_robot_id].heading = -atan2(y,x) + my_position.heading; // find the relative theta;
		range = sqrt((1/message_rssi));
		
		// Get position update

		relative_pos[other_robot_id].x = range*cos(relative_pos[other_robot_id].heading);  // relative x pos
		relative_pos[other_robot_id].y = -1.0 * range*sin(relative_pos[other_robot_id].heading);   // relative y pos
		
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
		wb_receiver_next_packet(receiver);
		//printf("relative position of the robot is %f and for y it is %f and finally the theta is %f\n", relative_pos[0].x, relative_pos[0].y, relative_pos[0].heading);
}
}*/

void receive_ping_messages()
{
    const double *message_direction;
    double message_rssi;  // Received Signal Strength indicator
	char *inbuffer; // Buffer to receive the sender id
    int sender_id, sender_flock_id, sender_flocking_group;

    double theta, range;
    double dx, dy, rx, ry;

    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver);
        sender_id = atoi(inbuffer);
        sender_flock_id = sender_id % FLOCK_SIZE;
        sender_flocking_group = sender_id / FLOCK_SIZE;

        if (sender_flocking_group != flocking_group) {
            wb_receiver_next_packet(receiver);
            continue;
        }

        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);

        dx = -message_direction[2];
        dy = -message_direction[0];

        flock_rpos[sender_flock_id].heading = atan2(dy, dx) + my_position.heading + M_PI / 2; // find the relative angle
        range = sqrt(1 / message_rssi);

        rx = range * cos(theta);        // relative x pos
        ry = range * sin(theta);        // relative y pos
        
        flock_rpos[sender_flock_id].x = rx;        // rel x pos
        flock_rpos[sender_flock_id].y = ry;        // rel y pos

        wb_receiver_next_packet(receiver);
    }
}

void update_self_motion(int msl, int msr) { 
	float theta = my_position.heading;
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position.x += dx;
	my_position.y += dz;
	my_position.heading += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position.heading > 2*M_PI) my_position.heading -= 2.0*M_PI;
	if (my_position.heading < 0) my_position.heading += 2.0*M_PI;
}

int main(){
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	int time_step = wb_robot_get_basic_time_step();
	int i;

	reset(time_step);                          // Initialization 
	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64); 

	msl=0; msr=0;  
	for(;;){
		
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  float test = wb_distance_sensor_get_value(ds[sensor_nb]);
		  distances[sensor_nb] = (int)test;
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds
		update_self_motion(msl, msr);
		receive_ping_messages();
		new_leader_range = sqrt(flock_rpos[0].x*flock_rpos[0].x + flock_rpos[0].y*flock_rpos[0].y);
		new_leader_bearing = -atan2(flock_rpos[0].x,flock_rpos[0].y);
		update_leader_measurement(new_leader_range, new_leader_bearing);
		compute_wheel_speeds(0, 0, &msl, &msr);
		//printf("the wheel speed is for msl %f and for msr %f\n", msl, msr);
		msl += bmsl;
		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(dev_left_motor, msl_w);
		wb_motor_set_velocity(dev_right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		wb_robot_step(64);               // Executing the simulation for 64ms
	}
}  
  
