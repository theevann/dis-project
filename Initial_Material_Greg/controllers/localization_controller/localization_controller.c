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

#include "trajectories.h"
#include "odometry.h"
//#include "kalman.h"
#include "flocking.h"


#define VERBOSE_ENC false  // Print encoder values
#define VERBOSE_ACC false  // Print accelerometer values
#define VERBOSE_GPS false  // Print gps values


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
WbDeviceTag receiver;	
	



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



int main()
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
        
        //process_flocking(previous_position, pos, receiver, emitter, ds);

        //formation_control();

        // Use one of the two trajectories.
        trajectory_1(dev_left_motor, dev_right_motor);
        // trajectory_2(dev_left_motor, dev_right_motor);
    }
}
