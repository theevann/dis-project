#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#include "../const.h"
#include "odometry.h"
#include "kalman_acc.h"
#include "kalman_vel.h"


// Choose a localisation technique
#define ENCODER 1
#define ACCELEROMETER 2
#define GPS 3
#define KALMAN_ACC 4
#define KALMAN_VEL 5

#define LOCALIZATION_METHOD KALMAN_VEL


// Calibrate Accelerometer
#define ACC_CAL false      // Enable accelerometer calibration (robot does not move for TIME_CAL seconds)
#define TIME_CAL 5         // Accelerometer calibration time (robot does not move for TIME_CAL seconds)


// ##### TO CLEANUP LATER
#define sign(X) (2*(X >= 0) - 1)

#define COHESION_WEIGHT 0.15
#define MIGR_WEIGHT 0.2
#define DISPERSION_WEIGHT 0.4
#define DISPERSION_THRESHOLD 0.15

#define NB_SENSORS 8
#define FLOCK_SIZE 5
#define BIAS_SPEED 400
#define MAX_SPEED 800
#define MAX_SPEED_WEB 6.28  // Maximum speed webots
#define AXLE_LENGTH 0.052	// Distance between wheels of robot (meters)


double braiten_weights[16] = {0.5, 0.4, 0.3, 0.1, -0.1, -0.3, -0.4, -0.5, -0.5, -0.4, -0.3, -0.1, 0.1, 0.3, 0.4, 0.5};
double last_obstacle_avoidance = -100.;

// ##### END TO CLEANUP LATER


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

static position_t initial_pos, pos, speed;
static position_t flock_prev_pos[ROBOTS_N], flock_pos[ROBOTS_N];

const position_t all_initial_pos[ROBOTS_N] = {
    {-2.9, 0., -M_PI / 2},
    {-2.9, -0.1, -M_PI / 2},
    {-2.9, 0.1, -M_PI / 2},
    {-2.9, -0.2, -M_PI / 2},
    {-2.9, 0.2, -M_PI / 2},
};

double last_gps_time_sec = 0.0f;
static int robot_id, robot_flock_id;
int time_step;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;
WbDeviceTag receiver;
WbDeviceTag dist_sensor[NB_SENSORS];


void init_variables()
{
    time_step = wb_robot_get_basic_time_step();

    // Initialize the robot id
    char *robot_name;
    robot_name = (char *)wb_robot_get_name();
    sscanf(robot_name, "epuck%d", &robot_id);
    robot_flock_id = robot_id % FLOCK_SIZE;

    // Initialize the position
    initial_pos = all_initial_pos[robot_id];
    memcpy(&pos, &initial_pos, sizeof(initial_pos));

    // Initialize the gps measurements
    double gps_init[] = {pos.x, 0, -pos.y};
    memcpy(meas.gps, gps_init, sizeof(meas.gps));
    memcpy(meas.prev_gps, gps_init, sizeof(meas.gps));
}


void init_devices()
{
    dev_gps = wb_robot_get_device("gps");
    wb_gps_enable(dev_gps, 1000);

    dev_acc = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(dev_acc, time_step);

    dev_left_encoder = wb_robot_get_device("left wheel sensor");
    dev_right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(dev_left_encoder, time_step);
    wb_position_sensor_enable(dev_right_encoder, time_step);

    dev_left_motor = wb_robot_get_device("left wheel motor");
    dev_right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(dev_left_motor, INFINITY);
    wb_motor_set_position(dev_right_motor, INFINITY);
    wb_motor_set_velocity(dev_left_motor, 0.0);
    wb_motor_set_velocity(dev_right_motor, 0.0);

	receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver, time_step); // TODO: CHECK time_step

	char sensor_name[4] = "ps0";
	for (int i = 0; i < NB_SENSORS; i++)
	{
		dist_sensor[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(dist_sensor[i], time_step); // TODO: CHECK time_step
        sensor_name[2]++;
    }
}


void clamp(float *number, float limit)
{
	*number = (*number > limit) ? limit : (*number < -limit) ? -limit : *number;
}


float dist(float v1[2], float v2[2])
{
    return sqrtf(powf(v1[0] - v2[0], 2) + powf(v1[1] - v2[1], 2));
}


float dist_pos(position_t pos_1, position_t pos_2)
{
    float v_pos_1[2] = {pos_1.x, pos_1.y};
    float v_pos_2[2] = {pos_2.x, pos_2.y};
    return dist(v_pos_1, v_pos_2);
}


void vector_to_wheelspeed(float *msl, float *msr, float vx, float vy)
{
    // Compute wanted position in robots coordinate
    float heading = pos.heading + M_PI / 2;
    float x = vx * cosf(heading) + vy * sinf(heading);  // x in robot coordinates
    float y = vx * -sinf(heading) + vy * cosf(heading);  // y in robot coordinates

    float Ku = 0.3;                     // Forward control coefficient
    float Kw = 0.75;                     // Rotational control coefficient
    float range = sqrtf(x * x + y * y); // Distance to the wanted position
    float bearing = atan2(y, x);        // Orientation of the wanted position

    // Compute forward control
    // float u = Ku * range * cosf(bearing);
    float u = Ku * cosf(bearing);
    // Compute rotational control
    float w = Kw * bearing;

    // Convert to wheel speeds!
    *msl = (u - AXLE_LENGTH * w / 2.0) * 2000;
    *msr = (u + AXLE_LENGTH * w / 2.0) * 2000;

    // printf("Reynolds MSL: %f MSR: %f\n", *msl, *msr);
    // printf("rob x: %f, rob y: %f, rob theta: %f (%f)\n", pos.x, pos.y, pos.heading + M_PI / 2, pos.heading);
    // printf("x: %f, y: %f\n", x, y);
    // printf("range: %f, bearing: %f\n", range, bearing);
    // printf("u: %f, w: %f\n", u, w);
    // limit(msl, MAX_SPEED);
    // limit(msr, MAX_SPEED);
    // printf("After Reynolds MSL: %d MSR: %d\n", *msl, *msr);
}


void braitenberg(float* msl, float* msr)
{
    double distances[NB_SENSORS]; // array keeping the distance sensor readings

    for (int sensor_nb = 0; sensor_nb < NB_SENSORS; sensor_nb++)
    {
        distances[sensor_nb] = wb_distance_sensor_get_value(dist_sensor[sensor_nb]);
        *msl += (distances[sensor_nb] * (-2) * braiten_weights[sensor_nb]);
        *msr += (distances[sensor_nb] * (-2) * braiten_weights[sensor_nb + NB_SENSORS]);
    }

    // printf("Braitenberg MSL: %f MSR: %f\n", *msl, *msr);
}


void reynolds(float* msl, float* msr)
{
    // vector_to_wheelspeed(msl, msr, MIGR_X - pos.x, MIGR_Y - pos.y);

    position_t migr = {MIGR_X, MIGR_Y, 0};
	position_t flock_pos_avg = {0, 0, 0};	 // Flock average positions
	position_t dispersion = {0, 0, 0};	 // Flock average positions
	position_t speed = {0, 0, 0};	 // Flock average positions
	// float flock_spd_avg[2] = {0, 0}; // Flock average speeds

    float norm;

	for (int i = 0; i < FLOCK_SIZE; i++)
	{
		if (i == robot_flock_id)
			continue;

        flock_pos_avg.x += flock_pos[i].x;
        flock_pos_avg.y += flock_pos[i].y;
        // flock_spd_avg.x += flock_spd[i][0];
        // flock_spd_avg.x += flock_spd[i][1];
		
        norm = dist_pos(pos, flock_pos[i]);
        if (norm < DISPERSION_THRESHOLD)
        {
            dispersion.x += norm * sign(pos.x - flock_pos[i].x) / (abs(pos.x - flock_pos[i].x) + 0.5);
            dispersion.y += norm * sign(pos.y - flock_pos[i].y) / (abs(pos.y - flock_pos[i].y) + 0.5);
        }
	}

	flock_pos_avg.x /= (FLOCK_SIZE - 1);
	flock_pos_avg.y /= (FLOCK_SIZE - 1);
	// flock_spd_avg.x /= (FLOCK_SIZE - 1);
	// flock_spd_avg.y /= (FLOCK_SIZE - 1);


    // Cohesion
    norm = dist_pos(flock_pos_avg, pos);
    speed.x += (flock_pos_avg.x - pos.x) / norm * COHESION_WEIGHT;
    speed.y += (flock_pos_avg.y - pos.y) / norm * COHESION_WEIGHT;

    // printf("_\nCohesion %f, %f\n", speed.x, speed.y);

	// Dispersion
    speed.x += dispersion.x * DISPERSION_WEIGHT;
    speed.y += dispersion.y * DISPERSION_WEIGHT;


    // Migration
    norm = dist_pos(migr, pos);
    speed.x += (MIGR_X - pos.x) / norm * MIGR_WEIGHT;
    speed.y += (MIGR_Y - pos.y) / norm * MIGR_WEIGHT;


    // Update msl and msr
    vector_to_wheelspeed(msl, msr, speed.x, speed.y);

    if (robot_id == 10) {
        printf("Dispersion %f, %f\n", dispersion.x * DISPERSION_WEIGHT, dispersion.y * DISPERSION_WEIGHT);
        printf("Migration %f, %f\n", (MIGR_X - pos.x) / norm * MIGR_WEIGHT, (MIGR_Y - pos.y) / norm * MIGR_WEIGHT);
        printf("Reynold MSL: %f MSR: %f\n", *msl, *msr);
    }
}


void send_ping_message()
{
    char robot_id_str[4];
    sprintf(robot_id_str, "%.2d", robot_id);
    wb_emitter_send(emitter, robot_id_str, strlen(robot_id_str));
}


void receive_ping_messages()
{
    const double *message_direction;
    double message_rssi;  // Received Signal Strength indicator

    char *inbuffer; // Buffer to receive the sender id
    int sender_id, sender_flock_id;

    double theta, range;
    double dx, dy, rx, ry;

    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver);
        sender_id = atoi(inbuffer);
        sender_flock_id = sender_id % FLOCK_SIZE;

        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);

        dx = -message_direction[2];
        dy = -message_direction[0];

        theta = atan2(dy, dx) + pos.heading + M_PI / 2; // find the relative angle
        range = sqrt(1 / message_rssi);

        rx = range * cos(theta);        // relative x pos
        ry = range * sin(theta);        // relative y pos
        
        // printf("_\nReceive ping from robot %d, rx: %f, ry: %f\n", sender_id, rx, ry);
        // printf("theta: %f (%f), range: %f\n", theta, atan2(y, x), range);
        // printf("y %f, x %f\n", y, x);

        // Get position update
        flock_prev_pos[sender_flock_id].x = flock_pos[sender_flock_id].x;
        flock_prev_pos[sender_flock_id].y = flock_pos[sender_flock_id].y;

        flock_pos[sender_flock_id].x = rx + pos.x;        // absolute x pos
        flock_pos[sender_flock_id].y = ry + pos.y;        // absolute y pos

        // relative_speed[sender_id][0] = (1. / DELTA_T) * (flock_pos[sender_id].x - flock_prev_pos[sender_id].x);
        // relative_speed[sender_id][1] = (1. / DELTA_T) * (flock_pos[sender_id].y - flock_prev_pos[sender_id].y);

        wb_receiver_next_packet(receiver);
    }
}


int main()
{
    wb_robot_init();
    init_variables();
    init_devices();
    init_odometry(time_step);
    
    if (LOCALIZATION_METHOD == KALMAN_ACC)
        init_kalman_acc(&initial_pos);
    else if(LOCALIZATION_METHOD == KALMAN_VEL)
        init_kalman_vel(&initial_pos);

    while (wb_robot_step(time_step) != -1 && robot_id < FLOCK_SIZE)
    {
        // printf("\n \n");
        // printf("\nNEW TIMESTEP\n");

        // READ SENSORS
        get_encoder(dev_left_encoder, dev_right_encoder, &(meas.prev_left_enc), &(meas.prev_right_enc), &(meas.left_enc), &(meas.right_enc));
        get_acc(dev_acc, meas.acc);
        get_gps(dev_gps, &last_gps_time_sec, meas.prev_gps, meas.gps, &(meas.gps_true));

        // CALIBRATE ACCELEROMETER IF ENABLED
        double time_now_s = wb_robot_get_time();
        bool calibrating = ACC_CAL && (time_now_s < TIME_CAL);

        if (calibrating)
        {
            calibrate_acc(meas.acc, meas.acc_mean);
        }

        // UPDATE POSITION ESTIMATION
        if (LOCALIZATION_METHOD == ENCODER)
        {
            update_pos_enc(&pos, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == ACCELEROMETER && !calibrating)
        {
            update_pos_acc(&pos, &speed, meas.acc, meas.acc_mean, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == GPS)
        {
            update_pos_gps(&pos, &last_gps_time_sec, meas.prev_gps, meas.gps);
        }
        else if (LOCALIZATION_METHOD == KALMAN_ACC && !calibrating)
        {
            update_pos_kalman_acc(&pos, meas.acc, meas.gps, meas.gps_true);
            update_heading_enc(&(pos.heading), meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        }
        else if (LOCALIZATION_METHOD == KALMAN_VEL)
        {
            double velocities[] = {wb_motor_get_velocity(dev_right_motor), wb_motor_get_velocity(dev_left_motor)};
            update_pos_kalman_vel(&pos, velocities, meas.gps, meas.gps_true);
        }

	    // printf("(EST) Kalman with velocity is x: %g, y: %g, theta: %g\n", pos.x, pos.y, pos.heading);
        

        // SEND & RECEIVE PINGS
        send_ping_message();
        receive_ping_messages();


        // DEFINE WHEEL SPEED
        float msl, msr; // Motor speed left and right
        float msl_w, msr_w;
        float braiten_msl = 0, braiten_msr = 0;
        float reynolds_msl = 0, reynolds_msr = 0;

        braitenberg(&braiten_msl, &braiten_msr);
        reynolds(&reynolds_msl, &reynolds_msr);

        if (time_now_s - last_obstacle_avoidance < 2)
        {
            msl = braiten_msl + BIAS_SPEED;
            msr = braiten_msr + BIAS_SPEED;
        }
        else if (abs(braiten_msl) + abs(braiten_msr) > 350)
        {
            printf("Start obstacle avoidance\n");
            last_obstacle_avoidance = time_now_s;
            msl = braiten_msl + BIAS_SPEED;
            msr = braiten_msr + BIAS_SPEED;
        }
        else
        {
            msl = reynolds_msl + braiten_msl + BIAS_SPEED / 4;
            msr = reynolds_msr + braiten_msr + BIAS_SPEED / 4;
        }

        if (robot_id == 10)
            printf("Final MSL: %f MSR: %f\n", msl, msr);

        clamp(&msl, MAX_SPEED);
        clamp(&msr, MAX_SPEED);

        msl_w = msl * MAX_SPEED_WEB / 1000.; // TODO: 1000 should probably be MAX_SPEED
        msr_w = msr * MAX_SPEED_WEB / 1000.;

        wb_motor_set_velocity(dev_left_motor, msl_w);
        wb_motor_set_velocity(dev_right_motor, msr_w);
    }
}
