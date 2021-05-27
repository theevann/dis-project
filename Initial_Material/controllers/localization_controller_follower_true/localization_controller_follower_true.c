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
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define FLOCK_SIZE	  5	  // Size of flock
/*Webots 2018b*/



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

float goal_range   = 0.0;
float goal_bearing = 0.0;

float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;
double theta = 0.0;

position_t relative_pos[FLOCK_SIZE];	// relative X, Z, Theta of all robots
position_t my_position;     		// X, Z, Theta of the current robot

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
	x += goal_range * cosf(- M_PI + goal_bearing + theta);
	y += goal_range * sinf(- M_PI + goal_bearing + theta);
	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float w = Kw * range * sinf(bearing) + Kb * leader_orientation;
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)

	// Convert to wheel speeds!
	*msl = (int)((u - AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = (int)((u + AXLE_LENGTH*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
}



void process_received_ping_messages(){
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

		theta =	-atan2(y,x);
		theta = theta + my_position.heading; // find the relative theta;
		range = sqrt((1/message_rssi));
		

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		// Get position update

		relative_pos[other_robot_id].x = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id].y = -1.0 * range*sin(theta);   // relative y pos
		
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
		wb_receiver_next_packet(receiver);
}
}

int main(){
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int time_step = wb_robot_get_basic_time_step();
	int i,initialized;

	reset(time_step);                          // Initialization 

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64); 

	//read the initial packets
	//initialized = 0;
			/* Wait until supervisor sent range and bearing information */

	/*while(!initialized){
		while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(64); // Executing the simulation for 64ms
		}  
		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  %d \n", (int)rbbuffer[0]);
			if((int)rbbuffer[0]==robot_id){
				initialized = 1;
				rbbuffer = (float*) wb_receiver_get_data(receiver);
				goal_range = sqrt(rbbuffer[1]*rbbuffer[1] + rbbuffer[2]*rbbuffer[2]);
				goal_bearing = -atan2(rbbuffer[1],rbbuffer[2]);
				printf("Goal of robot %d: range = %.2f, bearing = %.2f\n", robot_id, goal_range, goal_bearing);
				leader_range = goal_range;
				leader_bearing = goal_bearing;
				leader_orientation = rbbuffer[2];
				printf("  Initial relative position: range = %.2f, bearing = %.2f, heading = %.2f\n", leader_range, leader_bearing, leader_orientation);
			}
			wb_receiver_next_packet(receiver);
		}
	}*/
	msl=0; msr=0;  
	for(;;){
		
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds
		process_received_ping_messages();
		new_leader_range = sqrt(relative_pos[0].x*relative_pos[0].x + relative_pos[0].y*relative_pos[0].y);
		new_leader_bearing = -atan2(relative_pos[0].x,relative_pos[0].y);
		update_leader_measurement(new_leader_range, new_leader_bearing);


		compute_wheel_speeds(0, 0, &msl, &msr);

		msl += bmsl;
		msr += bmsr;

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
  
