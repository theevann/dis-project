#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/emitter.h>


#include "trajectories.h"
#include "odometry.h"
#include "kalman_acc.h"
#include "kalman_vel.h"

#define VERBOSE_ENC false  // Print encoder values
#define VERBOSE_ACC false  // Print accelerometer values
#define VERBOSE_GPS false  // Print gps values

#define KALMAN_ACC false
#define ACC_CAL false      // Enable accelerometer calibration ( robot does not move for 5 seconds )

#define ROBOTS_N 1

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
const position_t initial_pos = { -2.9, 0., -M_PI/2 }; // absolute position theta is -pi/2
static int robot_id;
double last_gps_time_sec = 0.0f;
bool gps_initialised = false;
static double acc_mean_sum[2] = {0., 0.};
static double acc_cal_tsum = 0;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;
WbDeviceTag emitter;		

/**

void test_gsl() {
    double A_data[] = {
        0.57092943, 0.00313503, 0.88069151, 0.39626474,
        0.33336008, 0.01876333, 0.12228647, 0.40085702,
        0.55534451, 0.54090141, 0.85848041, 0.62154911,
        0.64111484, 0.8892682 , 0.58922332, 0.32858322
    };

    double b_data[] = {
        1.5426693 , 0.74961678, 2.21431998, 2.14989419
    };

    // Access the above C arrays through GSL views
    gsl_matrix_view A = gsl_matrix_view_array(A_data, 4, 4);
    gsl_vector_view b = gsl_vector_view_array(b_data, 4);

    // Print the values of A and b using GSL print functions
    printf("A = \n");
    gsl_matrix_fprintf (stdout, &A.matrix, "%lf");

    printf("\nb = \n");
    gsl_vector_fprintf (stdout, &b.vector, "%lf");

    // Allocate memory for the solution vector x and the permutation perm:
    gsl_vector *x = gsl_vector_alloc (4);
    gsl_permutation *perm = gsl_permutation_alloc (4);

    // Decompose A into the LU form:
    int signum;
    gsl_linalg_LU_decomp (&A.matrix, perm, &signum);

    // Solve the linear system
    gsl_linalg_LU_solve (&A.matrix, perm, &b.vector, x);

    // Print the solution
    printf("\nx = \n");
    gsl_vector_fprintf (stdout, x, "%lf");

    // Release the memory previously allocated for x and perm
    gsl_vector_free(x);
    gsl_permutation_free(perm);
}

/**/

void init_pos(position_t *pos)
{
    memcpy(pos, &initial_pos, sizeof(initial_pos));
}


void init_devices(int ts)
{
    char* robot_name; 
	robot_name=(char*) wb_robot_get_name(); 
    sscanf(robot_name,"ROBOT%d",&robot_id);
    
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
    
    emitter = wb_robot_get_device("emitter");
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


void calibrate_acc() 
{    
    double time_now_s = wb_robot_get_time();
    if (time_now_s < 5 && time_now_s > 0.1) {
        acc_cal_tsum += 1;
        acc_mean_sum[0] += meas.acc[0]; 
        acc_mean_sum[1] += meas.acc[1]; 
        meas.acc_mean[0] = acc_mean_sum[0] / acc_cal_tsum;
        meas.acc_mean[1] = acc_mean_sum[1] / acc_cal_tsum;
        printf("Acc mean: %g %g %g\n", meas.acc_mean[0], meas.acc_mean[1], meas.acc_mean[2]);
    }
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
    meas.gps_true = false;

    if (!gps_initialised) {
        meas.gps[0] = initial_pos.x;
        meas.gps[1] = 0;
        meas.gps[2] = initial_pos.y;
        memcpy(meas.prev_gps, meas.gps, sizeof(meas.gps));
        meas.gps_true = true;
        gps_initialised = true;
    } else if (time_now_s - last_gps_time_sec > 1) {
        last_gps_time_sec = time_now_s;
        memcpy(meas.prev_gps, meas.gps, sizeof(meas.gps));
        memcpy(meas.gps, wb_gps_get_values(dev_gps), sizeof(meas.gps));
        meas.gps_true = true;
        
        if (VERBOSE_GPS)
            printf("ROBOT absolute gps : %g %g %g\n", meas.gps[0], meas.gps[1], meas.gps[2]);
    }
    
}


void update_pos_gps(position_t *pos)
{
    double time_now_s = wb_robot_get_time();
    double delta_x =   meas.gps[0] - meas.prev_gps[0];
    double delta_y = -(meas.gps[2] - meas.prev_gps[2]);

    pos->x =  meas.gps[0] + (time_now_s - last_gps_time_sec) * delta_x;
    pos->y = -meas.gps[2] + (time_now_s - last_gps_time_sec) * delta_y;
    pos->heading = atan2(delta_y, delta_x);

    if (VERBOSE_GPS)
        printf("GPS Relative est.: %g %g %g\n", pos->x, pos->y, pos->heading);
}


void send_position(position_t pos)
{
    char buffer[255]; // Buffer for sending data

    // Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it
    sprintf(buffer, "%d#%g#%g#%g", robot_id, pos.x, pos.y, pos.heading);
    wb_emitter_send(emitter, buffer, strlen(buffer));
}


int main()
{
    wb_robot_init();
    int time_step = wb_robot_get_basic_time_step();
    init_pos(&pos);
    init_devices(time_step);
    init_odometry(time_step);

    if (KALMAN_ACC)
        init_kalman_acc(&initial_pos);
    else
        init_kalman_vel(&initial_pos);

    while (wb_robot_step(time_step) != -1)
    {
        printf("\n \n");
        printf("\nNEW TIMESTEP\n");

        // READ SENSORS
        controller_get_encoder();
        controller_get_acc();
        controller_get_gps();

        // CALIBRATE ACCELEROMETER IF ENABLED
        if (ACC_CAL) {
            calibrate_acc();
        }

        // UPDATE POSITION ESTIMATION
        // update_pos_odo_enc(&pos, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        // update_pos_odo_acc(&pos, &speed, meas.acc, meas.acc_mean, meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        // update_pos_gps(&pos);
        
        //*
        if (KALMAN_ACC) {
            update_pos_kalman_acc(&pos, meas.acc, meas.gps, meas.gps_true);
            update_heading_enc(&(pos.heading), meas.left_enc - meas.prev_left_enc, meas.right_enc - meas.prev_right_enc);
        } else {
            double velocities[] = {wb_motor_get_velocity(dev_right_motor), wb_motor_get_velocity(dev_left_motor)};
            update_pos_kalman_vel(&pos, velocities, meas.gps, meas.gps_true);
        }
        //*/

        // Send the estimated position to the supervisor for metric computation
        send_position(pos);

        // Use one of the two trajectories.
        trajectory_1(dev_left_motor, dev_right_motor, ACC_CAL);
        // trajectory_2(dev_left_motor, dev_right_motor, ACC_CAL);
    }
}
