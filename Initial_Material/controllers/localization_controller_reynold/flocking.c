#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

//#include "flocking.h"

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  4	  // Size of flock

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.3/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


//WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
position_t relative_pos[FLOCK_SIZE];	// relative X, Z, Theta of all robots
position_t prev_relative_pos[FLOCK_SIZE];	// Previous relative  X, Z, Theta values
position_t my_position;     		// X, Z, Theta of the current robot
position_t prev_my_position;  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,-25};	       // Migration vector
char* robot_name;
float theta_robots[FLOCK_SIZE];

void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

void reynolds_rules(){
    int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};


	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id) {
			continue;
		}

		for (j=0;j<2;j++) {
			switch (j)
					{
					case 0:
						rel_avg_speed[j] += relative_speed[i][0];
						rel_avg_loc[j] += relative_pos[i].x;
						break;
					case 1:
						rel_avg_speed[j] += relative_speed[i][1];
						rel_avg_loc[j] += relative_pos[i].y;
						break;
					default:
						break;
					}
		}
	}
	for (j=0;j<2;j++) {
		rel_avg_speed[j] /= FLOCK_SIZE-1;
		rel_avg_loc[j] /= FLOCK_SIZE-1;
	}
	for (j=0;j<2;j++) {
		cohesion[j] = rel_avg_loc[j];
	}
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id){ 
			if(pow(relative_pos[k].x,2)+pow(relative_pos[k].y,2) < RULE2_THRESHOLD){
				for (j=0;j<2;j++) {
					switch (j)
					{
					case 0:
						dispersion[j] -= 1/relative_pos[k].x;
						break;
					case 1:
						dispersion[j] -= 1/relative_pos[k].y;
						break;
					default:
						break;
					}
				}
			}
		}
	}
	for (j=0;j<2;j++){
		consistency[j] = rel_avg_speed[j];
	}
    
	//aggregation of all behaviors with relative influence determined by weights
	for (j=0;j<2;j++) {
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	speed[robot_id][1] *= -1; //y axis of webots is inverted
        
	if(MIGRATORY_URGE == 0){
	  speed[robot_id][0] += 0.01*cos(my_position.heading + M_PI/2);
	  speed[robot_id][1] += 0.01*sin(my_position.heading + M_PI/2);
	} else {
		speed[robot_id][0] += (migr[0]-my_position.x) * MIGRATION_WEIGHT;
		speed[robot_id][1] -= (migr[1]-my_position.y) * MIGRATION_WEIGHT; //y axis of webots is inverted
	}
}

void compute_wheels_speed(int *msl, int *msr){
    float x = speed[robot_id][0]*cosf(my_position.heading) + speed[robot_id][1]*sinf(my_position.heading); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position.heading) + speed[robot_id][1]*cosf(my_position.heading); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	float u = Ku*range*cosf(bearing);
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

void send_ping(void){
    char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter2,out,strlen(out)+1); 
}
void process_received_ping_messages(){
    const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver2) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver2);
		message_direction = wb_receiver_get_emitter_direction(receiver2);
		message_rssi = wb_receiver_get_signal_strength(receiver2);
		double y = message_direction[2];
		double x = message_direction[1];

		theta =	-atan2(y,x);
		theta = theta + my_position.heading; // find the relative theta;
		range = sqrt((1/message_rssi));
		

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		// Get position update
		prev_relative_pos[other_robot_id].x = relative_pos[other_robot_id].x;
		prev_relative_pos[other_robot_id].y = relative_pos[other_robot_id].y;

		relative_pos[other_robot_id].x = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id].y = -1.0 * range*sin(theta);   // relative y pos
	
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id].x-prev_relative_pos[other_robot_id].x);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id].y-prev_relative_pos[other_robot_id].y);		
		 
		wb_receiver_next_packet(receiver2);
}
}

void process_flocking(position_t previous_position, position_t current_position, WbDeviceTag receiver, WbDeviceTag emitter, WbDeviceTag* des){
	receiver2 = receiver;
	emitter2 = emitter; 
	int msl, msr;			// Wheel speeds
	float msl_w, msr_w;
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	prev_my_position = previous_position;
	my_position = current_position;
    bmsl = 0; bmsr = 0;
	sum_sensors = 0;
	max_sens = 0;

    for(int i=0;i<8;i++)
		wb_distance_sensor_enable(des[i],64);  

	/* Braitenberg */
	for(i=0;i<NB_SENSORS;i++) {
		distances[i]=wb_distance_sensor_get_value(des[i]); //Read sensor values
		sum_sensors += distances[i]; // Add up sensor values
		max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value
		// Weighted sum of distance sensor values for Braitenburg vehicley
		bmsr += e_puck_matrix[i] * distances[i];
		bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i]; //TO DO: See what is this e-puck matrix
	}
		// Adapt Braitenberg values (empirical tests)
	bmsl/=MIN_SENS; bmsr/=MIN_SENS; // TO DO : check if the values are correct
	bmsl+=66; bmsr+=72;
             
	/* Send and get information */
	send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

	
	process_received_ping_messages();
	speed[robot_id][0] = (1/DELTA_T)*(my_position.x-prev_my_position.x);
	speed[robot_id][1] = (1/DELTA_T)*(my_position.y-prev_my_position.y);
   
	// Reynold's rules with all previous info (updates the speed[][] table)
	reynolds_rules();
   
	// Compute wheels speed from reynold's speed
	compute_wheels_speed(&msl, &msr);
   
	// Adapt speed instinct to distance sensor values
	if (sum_sensors > NB_SENSORS*MIN_SENS) {
		msl -= msl*max_sens/(2*MAX_SENS);
		msr -= msr*max_sens/(2*MAX_SENS);
	}
   
	// Add Braitenberg
	msl += bmsl;
	msr += bmsr;
                 
	// Set speed
	msl_w = msl*MAX_SPEED_WEB/1000;
	msr_w = msr*MAX_SPEED_WEB/1000;
	wb_motor_set_velocity(left_motor, msl_w);
	wb_motor_set_velocity(right_motor, msr_w);  
}
