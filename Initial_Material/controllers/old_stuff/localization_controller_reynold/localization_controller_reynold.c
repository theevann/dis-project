#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#include "trajectories.h"
#include "odometry.h"
//#include "kalman.h"


#define VERBOSE_ENC false  // Print encoder values
#define VERBOSE_ACC false  // Print accelerometer values
#define VERBOSE_GPS true  // Print gps values

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)
#define TIME_STEP	  64	  // [ms] Length of time step


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.3/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT       (1.0/50)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))


typedef struct 
{
    double prev_gps[3];
    double gps[3];
    double acc_mean[3];
    double acc[3];
    double prev_left_enc;
    double left_enc;
    double prev_right_enc;
    double right_enc;
    bool gps_true;
} measurement_t;


static measurement_t meas;
static position_t pos;
double good_rp[4][2] = { {0.0,0.0}, {0.0,-0.30}, {-0.15,-0.15}, {0.15,-0.15} };
const position_t initial_pos = { -2.9, 0., -M_PI/2 }; // absolute position theta is -pi/2
double last_gps_time_sec = 0.0f;
double loc[4][4];
int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
position_t relative_pos[FLOCK_SIZE];	// relative X, Z, Theta of all robots
position_t prev_relative_pos[FLOCK_SIZE];	// Previous relative  X, Z, Theta values
position_t my_position;     		// X, Z, Theta of the current robot
position_t prev_my_position;  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float speed_reynolds[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,-25};	       // Migration vector
char* robot_name;
float theta_robots[FLOCK_SIZE];
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


WbDeviceTag ds[8];	// Handle for the infrared distance sensors
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
	



void init_devices(int ts)
{
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
	for(i=0; i<8;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 
    //printf("The name of the epuck is 1 %s\n", robot_name);
	for(i=0;i<8;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64);
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

}


void controller_get_encoder()
{
    meas.prev_left_enc = meas.left_enc; // Store previous value
    meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
    
    meas.prev_right_enc = meas.right_enc; // Store previous value
    meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

    if (VERBOSE_ENC)
        printf("ROBOT enc : %g %g\n", meas.left_enc, meas.right_enc);
}


void controller_get_acc()
{
    memcpy(meas.acc, wb_accelerometer_get_values(dev_acc), sizeof(meas.acc));

    if (VERBOSE_ACC)
        printf("ROBOT acc : %g %g %g\n", meas.acc[0], meas.acc[1] , meas.acc[2]);
}


void controller_get_gps()
{
    double time_now_s = wb_robot_get_time();

    if (time_now_s - last_gps_time_sec > 1) {
        last_gps_time_sec = time_now_s;
        memcpy(meas.prev_gps, meas.gps, sizeof(meas.gps));
        memcpy(meas.gps, wb_gps_get_values(dev_gps), sizeof(meas.gps));
        meas.gps_true = true;
        
        if (VERBOSE_GPS)
           printf("ROBOT absolute gps : %g %g %g\n", meas.gps[0], meas.gps[1], meas.gps[2]);
    }
    else {
        meas.gps_true = false;
    }
}


void update_pos_gps(position_t *pos)
{
    double time_now_s = wb_robot_get_time();
    double delta_x =   meas.gps[0] - meas.prev_gps[0];
    double delta_y = -(meas.gps[2] - meas.prev_gps[2]);

    pos->x =   meas.gps[0] - initial_pos.x  + (time_now_s - last_gps_time_sec) * delta_x;
    pos->y = -(meas.gps[2] - initial_pos.y) + (time_now_s - last_gps_time_sec) * delta_y;
    pos-> heading = atan2(delta_y, delta_x);
	//printf("the position is x : %f and the y : and the theta %f",pos->x, pos->y, pos->heading);

    if (VERBOSE_GPS)
        printf("GPS : %g %g %g\n", pos->x, pos->y, pos->heading);
}


void send_position(position_t pos)
{
    char buffer[255]; // Buffer for sending data

    // Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it
    sprintf(buffer, "%g#%g#%g", pos.x, pos.y, pos.heading);
    wb_emitter_send(emitter, buffer, strlen(buffer));
}

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
	//cohesion
	for (j=0;j<2;j++) {
		cohesion[j] = (float)rel_avg_loc[j];
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
		speed_reynolds[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed_reynolds[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed_reynolds[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	speed_reynolds[robot_id][1] *= -1; //y axis of webots is inverted
        
	if(MIGRATORY_URGE == 0){
	  speed_reynolds[robot_id][0] += 0.01*cos(my_position.heading + M_PI/2);
	  speed_reynolds[robot_id][1] += 0.01*sin(my_position.heading + M_PI/2);
	} else {
		speed_reynolds[robot_id][0] += (migr[0]-my_position.x) * MIGRATION_WEIGHT;
		speed_reynolds[robot_id][1] -= (migr[1]-my_position.y) * MIGRATION_WEIGHT; //y axis of webots is inverted
	}
}

void compute_wheels_speed(int *msl, int *msr){
    float x = speed_reynolds[robot_id][0]*cosf(my_position.heading) + speed_reynolds[robot_id][1]*sinf(my_position.heading); // x in robot coordinates
	float z = -speed_reynolds[robot_id][0]*sinf(my_position.heading) + speed_reynolds[robot_id][1]*cosf(my_position.heading); // z in robot coordinates

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
	//strcpy(out,robot_name);  // in the ping message we send the name of the robot.
    int i = 0;
    for (i = 0; robot_name[i] != '\0'; ++i) {
        out[i] = robot_name[i];
    }
    out[i] = '\0';
	wb_emitter_send(emitter,out,strlen(out)+1); 
}
void process_received_ping_messages(){
    const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
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
		prev_relative_pos[other_robot_id].x = relative_pos[other_robot_id].x;
		prev_relative_pos[other_robot_id].y = relative_pos[other_robot_id].y;

		relative_pos[other_robot_id].x = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id].y = -1.0 * range*sin(theta);   // relative y pos
	
		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id].x-prev_relative_pos[other_robot_id].x);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id].y-prev_relative_pos[other_robot_id].y);		
		 
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
	int msl, msr;			// Wheel speeds
	float msl_w, msr_w;
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
    bmsl = 0; bmsr = 0;
	sum_sensors = 0;
	msl = 0; msr = 0; 
	max_sens = 0; 
 	position_t previous_position;
    wb_robot_init();
    int time_step = wb_robot_get_basic_time_step();
    init_devices(time_step);
    init_odometry(time_step);
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;

		controller_get_encoder();
        controller_get_acc();
        controller_get_gps();
		//update_pos_gps(&my_position);
		        //send_position(pos);


		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		/// Compute self position
		prev_my_position.x = my_position.x;
		prev_my_position.y = my_position.y;
		prev_my_position.heading = my_position.heading;
		
		update_self_motion(msl,msr);
		//printf("the position is x : %f and the y : %f and the theta %f\n",my_position.x,my_position.y, my_position.heading);
		//printf("the previous position is x : %f and the y : %f and theta %f\n", prev_my_position.x, prev_my_position.y, prev_my_position.heading);
		process_received_ping_messages();

		speed[robot_id][0] = (1/DELTA_T)*(my_position.x-prev_my_position.x);
		speed[robot_id][1] = (1/DELTA_T)*(my_position.y-prev_my_position.y);
		//printf("the speed is : %f \n", speed[robot_id][0]);
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
        //printf("the value of msl is %d and the value of msr is %d ...and the value of max speed is %f\n", msl, msr, (float)MAX_SPEED_WEB/(float)1000);          
		// Set speed
		msl_w = (float)msl*MAX_SPEED_WEB/(float)1000;
		msr_w = (float)msr*(float)MAX_SPEED_WEB/(float)1000;
		//printf("the value of msl_w is %f and the value of msr_w is %f\n", msl_w, msr_w);
		wb_motor_set_velocity(dev_left_motor, msl_w);
		wb_motor_set_velocity(dev_right_motor, msr_w);

    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  

/*int main()
{
    position_t previous_position;
    wb_robot_init();
    int time_step = wb_robot_get_basic_time_step();
    init_devices(time_step);
    init_odometry(time_step);
    //init_kalman(initial_pos.x, initial_pos.y);
    

    while (wb_robot_step(time_step) != -1)
    {
        
        // READ SENSORS
        controller_get_encoder();
        controller_get_acc();

        controller_get_gps();

        previous_position.x = pos.x;
        previous_position.y = pos.y;
        previous_position.heading = pos.heading;

        // UPDATE POSITION ESTIMATION
        // update_pos_odo_enc(&pos, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        // update_pos_odo_acc(&pos, &speed, meas.acc, meas.acc_mean, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
         update_pos_gps(&pos);

        //update_pos_kalman(&pos, meas.acc, meas.gps, meas.gps_true);
        
        // Send the estimated position to the supervisor for metric computation
        //send_position(pos);
        //formation_control();
        process_flocking(previous_position, pos);


        // Use one of the two trajectories.
        //trajectory_1(dev_left_motor, dev_right_motor);
        // trajectory_2(dev_left_motor, dev_right_motor);
    }
}*/

