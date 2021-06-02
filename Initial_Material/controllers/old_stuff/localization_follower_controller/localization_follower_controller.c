#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>


#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>

#include "trajectories.h"
#include "odometry.h"
//#include "kalman.h"
//#include "flocking.h"


#define NB_SENSORS           8

#define VERBOSE_ENC false  // Print encoder values
#define VERBOSE_ACC false  // Print accelerometer values
#define VERBOSE_GPS false  // Print gps values

/* Formation flocking parameters */
#define D                    0.20      // Distance between robots
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

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
static position_t pos, speed;
double good_rp[4][2] = { {0.0,0.0}, {0.0,-0.30}, {-0.15,-0.15}, {0.15,-0.15} };
const position_t initial_pos = { -2.9, 0., -M_PI/2 }; // absolute position theta is -pi/2
double last_gps_time_sec = 0.0f;
static WbFieldRef robs_translation[4];
static WbFieldRef robs_rotation[4];
double loc[4][4];

WbDeviceTag ds[8];	// Handle for the infrared distance sensors
WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;		
//WbDeviceTag receiver;	
	

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};	
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag receiver;                 // Handle for the receiver node for range and bearing information


int robot_id;                       // Unique robot ID

float goal_range   = 0.0;
float goal_bearing = 0.0;

float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;

static void reset(void) {
	wb_robot_init();

	receiver    = wb_robot_get_device("receiver(1)");

	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	/*Webots 2018b*/

	int i;

	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);      // the device name is specified in the world file
		s[2]++;                         // increases the device number
	}
	char* robot_name; robot_name=(char*) wb_robot_get_name(); 

	sscanf(robot_name,"e-puck%d",&robot_id);    // read robot id from the robot's name
	printf("Reset: robot %d\n",robot_id);
}

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
    char* robot_name;
    int i;
	char s[4]="ps0";
	for(i=0; i<8;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<8;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64);
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

void update_leader_measurement(float new_leader_range, float new_leader_bearing, float new_leader_orientation) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
	leader_orientation = new_leader_orientation;
}

void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr) {
	// Define constants
	float Ku = 2.0;
	float Kw = 10.0;
	float Kb = 1.0;

	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	float theta = leader_orientation;
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

void formation_follower()
{

    int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int i,initialized;

	reset();                          // Initialization 
	//init_devices();

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64); 
	
	//read the initial packets
	initialized = 0;
	while(!initialized){
		/* Wait until supervisor sent range and bearing information */
		while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(64); // Executing the simulation for 64ms
		}  
		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
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
	}
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

		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
			if((int)rbbuffer[0]==robot_id){
				new_leader_range = sqrt(rbbuffer[1]*rbbuffer[1] + rbbuffer[2]*rbbuffer[2]);
				new_leader_bearing = -atan2(rbbuffer[1],rbbuffer[2]);
				new_leader_orientation = rbbuffer[3];
				update_leader_measurement(new_leader_range, new_leader_bearing, new_leader_orientation);
				//printf("%f: Leader: r:%f gr:%f, b:%f gb:%f, o:%f\n", new_leader_range, goal_range, new_leader_bearing, goal_bearing, new_leader_orientation);
			}
			wb_receiver_next_packet(receiver);
		}


		compute_wheel_speeds(0, 0, &msl, &msr);

		msl += bmsl;
		msr += bmsr;

		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		wb_robot_step(64);               // Executing the simulation for 64ms
	}


}


int main()
{
    /*position_t previous_position;
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
        
        //process_flocking(previous_position, pos, receiver, emitter, ds);

        //formation_control();

        // Use one of the two trajectories.
        //trajectory_1(dev_left_motor, dev_right_motor);
        // trajectory_2(dev_left_motor, dev_right_motor);

        //

        formation_follower();
		*/
	printf("checkpoint 1\n");
	int msl,msr;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int i,initialized;

	reset();                          // Initialization 
	//init_devices();

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64); 
	
	//read the initial packets
	initialized = 0;
	while(!initialized){
		/* Wait until supervisor sent range and bearing information */
		printf("wait wait wait\n");
		while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(64); // Executing the simulation for 64ms
		}  
		printf("found found found\n");
		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
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
	}
	printf("checkpoint 2\n");
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

		if (wb_receiver_get_queue_length(receiver) > 0) {
			rbbuffer = (float*) wb_receiver_get_data(receiver);
			if((int)rbbuffer[0]==robot_id){
				new_leader_range = sqrt(rbbuffer[1]*rbbuffer[1] + rbbuffer[2]*rbbuffer[2]);
				new_leader_bearing = -atan2(rbbuffer[1],rbbuffer[2]);
				new_leader_orientation = rbbuffer[3];
				update_leader_measurement(new_leader_range, new_leader_bearing, new_leader_orientation);
				//printf("%d: Leader: r:%f gr:%f, b:%f gb:%f, o:%f\n", new_leader_range, goal_range, new_leader_bearing, goal_bearing, new_leader_orientation);
			}
			wb_receiver_next_packet(receiver);
		}


		compute_wheel_speeds(0, 0, &msl, &msr);

		msl += bmsl;
		msr += bmsr;

		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		wb_robot_step(64);               // Executing the simulation for 64ms
	}
}
