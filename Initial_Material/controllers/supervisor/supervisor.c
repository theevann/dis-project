/*
 * File:          supervisor_local.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#include "../localization_controller/odometry.h"

#define ROBOTS_N 1
#define TASK 2            // Tasks: 0 is localization, 1 is flocking, 2 is formation control
#define LEADER_ID 0
#define VERBOSE_METRIC true // print error on each timestep
#define VERBOSE_SUPPOS false    // print position from supervisor

WbNodeRef rob[ROBOTS_N];           // Robot node
WbFieldRef rob_trans[ROBOTS_N];    // Robots translation fields
WbFieldRef rob_rotation[ROBOTS_N]; // Robots rotation fields
WbDeviceTag receiver;    // Single receiver

static int time_step;
static float t = 0.0;
float loc_abs[ROBOTS_N][3]; // Absolute Location of the robot
float loc_est[ROBOTS_N][3]; // Estimated position
bool saved = false;
double metric_loc = 0.0;
double metric_flock = 0.0;
double metric_form = 0.0;
long metric_stepcount = 0;
const position_t initial_pos = {-2.9, 0., -M_PI / 2};

static float D_flock = 0.5;         // size [ROBOTS_N][ROBOTS_N] ? TO BE DEFINED
static const float D_max_flock = 2; // TO BE DEFINED
static float prev_fcenter[2] = {-1000, -1000};
static const float rel_formation_pos[ROBOTS_N][2] = {{0., 0.}}; // TO BE DEFINED
static const float D_max_form = 2;                 // TO BE DEFINED

/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super(void)
{
    wb_robot_init();

    t = 0.0;
    time_step = wb_robot_get_basic_time_step();

    receiver = wb_robot_get_device("receiver");
    if (receiver == 0)
        printf("missing receiver in supervisor\n");

    wb_receiver_enable(receiver, time_step);

    int i;
    char rob_name[7];
    for (i=0;i<ROBOTS_N;i++) {
        sprintf(rob_name, "ROBOT%d", i+1); // verify
        rob[i] = wb_supervisor_node_get_from_def(rob_name);
        rob_trans[i] = wb_supervisor_node_get_field(rob[i], "translation");
        rob_rotation[i] = wb_supervisor_node_get_field(rob[i], "rotation");
    }
}

void get_absolute_position(void)
{
    // Get data
    int i;
    for (i=0;i<ROBOTS_N;i++) {
        loc_abs[i][0] = wb_supervisor_field_get_sf_vec3f(rob_trans[i])[0];        // X
        loc_abs[i][1] = -wb_supervisor_field_get_sf_vec3f(rob_trans[i])[2];       // Z
        loc_abs[i][2] = wb_supervisor_field_get_sf_rotation(rob_rotation[i])[3];  // THETA
    }
    
    if (VERBOSE_SUPPOS) {
        int robot_print = 0;
        printf("(GT) Position of robot%d is x: %f, y: %f, theta: %f\n", robot_print+1, loc_abs[robot_print][0], loc_abs[robot_print][1], loc_abs[robot_print][2]);
        // printf("(GT) vx: %f\n", wb_supervisor_node_get_velocity(rob)[0]);
      }

    
}

void get_info(void)
{
    float rob_x, rob_z, rob_theta;
    int rob_nb;
    char *inbuffer;

    int count = 0;
    while (wb_receiver_get_queue_length(receiver) > 0 && count < ROBOTS_N)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver);
        sscanf(inbuffer, "%d#%f#%f#%f", &rob_nb, &rob_x, &rob_z, &rob_theta);
        loc_est[rob_nb-1][0] = rob_x;
        loc_est[rob_nb-1][1] = rob_z;
        loc_est[rob_nb-1][2] = rob_theta;
        count++;
        wb_receiver_next_packet(receiver);
    }
}

// TO DO: add sending 0 from GPS when it is nan !

void compute_metric(void)
{
    // Position sent from robot
    // printf("Estimated position is x: %f, y: %f, theta: %f\n",loc_est[0],loc_est[1],loc_est[2]);

    // Compute the metric for the localization task
    if (TASK == 0)
    {
        if (ROBOTS_N == 1) { 
            if (t < 115.0)
            {
                float error;
                error = sqrt(pow((loc_abs[0][0] - loc_est[0][0]), 2) + pow((loc_abs[0][1] - loc_est[0][1]), 2));
                if (error == error)
                {
                    metric_loc += error;
                }
                if (VERBOSE_METRIC)
                {
                    printf("Localization Error: %f\n", error);
                }
            }
            else if (t > 115.0 && saved == false)
            {
                printf("Localization metric: %f\n", metric_loc);
                FILE *fp;
                fp = fopen("..\\..\\metric_scores\\localization.txt", "w");
                fprintf(fp, "Localization metric score: %f\n", metric_loc);
                fclose(fp);
                printf("Metric saved to localization.txt\n");
                saved = true;
            }
        }
        else {
            printf("Localization metric is not available for multiple robots");
        }
    }
    else if (TASK == 1)
    {
        if (t < 115.0)
        {
            float x_sum=0;
            float y_sum=0;
            float flock_center[2];
            float fit_o=0; 
            float fit_d1=0; 
            float fit_d2=0; 
            float fit_v=0;
            double fit_step;
            float dist_diff;
            int i; int j;
            // Compute center of flock
            for (i=0;i<ROBOTS_N;i++) {
                x_sum += loc_abs[i][0];
                y_sum += loc_abs[i][1];
            }
            flock_center[0] = x_sum / ROBOTS_N;
            flock_center[1] = y_sum / ROBOTS_N;
            for (i=0;i<ROBOTS_N;i++) {
                for (j=i+1;j<ROBOTS_N;j++) {
                    // Orientation measure for each pair of robots
                    fit_o += fabsf(loc_abs[i][2]-loc_abs[j][2]) / M_PI;
                    // Second part of distance measure for each pair of robots
                    dist_diff = sqrtf(powf(loc_abs[i][0]-loc_abs[j][0],2)+powf(loc_abs[i][1]-loc_abs[j][1],2));
                    fit_d2 += fminf((dist_diff/D_flock), (1/powf(1-D_flock+dist_diff, 2)));
                }
                // First part of distance measure for each pair of robots
                fit_d1 += sqrtf(powf(loc_abs[i][0]-flock_center[0],2)+powf(loc_abs[i][1]-flock_center[1],2));
            }
            // Speed measure
            if (prev_fcenter[0] == -1000 && prev_fcenter[1] == -1000) {
                prev_fcenter[0] = flock_center[0];
                prev_fcenter[1] = flock_center[1];}
            fit_v = sqrtf(powf(flock_center[0]-prev_fcenter[0],2)+powf(flock_center[1]-prev_fcenter[1],2)) / D_max_flock;
            // Total metric on this timestep
            fit_step = (1 - 2/(ROBOTS_N*(ROBOTS_N-1)) * fit_o) * ((2/(ROBOTS_N*(ROBOTS_N-1)) * fit_d2) / (1 + fit_d1/ROBOTS_N)) * fit_v;
            metric_flock += fit_step;
            metric_stepcount++;
            prev_fcenter[0] = flock_center[0];
            prev_fcenter[1] = flock_center[1];
            if (VERBOSE_METRIC)
                {
                    printf("Flocking Metric: %f\n", fit_step);
                }
        }
        else if (t > 115.0 && saved == false)
        {
            // Calculate mean of flocking metric
            double meanmet_flock;
            meanmet_flock = metric_flock / metric_stepcount;
            // Print and save to file
            printf("Flocking metric: %f\n", meanmet_flock);
            FILE *fp;
            fp = fopen("..\\..\\metric_scores\\flocking.txt", "w");
            fprintf(fp, "Flocking metric score: %f\n", meanmet_flock);
            fclose(fp);
            printf("Metric saved to flocking.txt\n");
            saved = true;
        }
    }
    else if (TASK == 2)
    {
        if (t < 115.0)
        {
            float x_sum=0;
            float y_sum=0;
            float flock_center[2];
            float fit_d=0;
            float fit_v=0;
            double fit_step;
            int i;
            float rob_formation_pos[2];
            for (i=0;i<ROBOTS_N;i++) {
                // Calculate each robot's goal position on this timestep
                rob_formation_pos[0] = loc_abs[LEADER_ID][0] + rel_formation_pos[i][0];
                rob_formation_pos[1] = loc_abs[LEADER_ID][1] + rel_formation_pos[i][1];
                // Distance measure to its goal position for each robot
                fit_d += sqrtf(powf(loc_abs[i][0]-rob_formation_pos[0],2)+powf(loc_abs[i][1]-rob_formation_pos[1],2));
                // Calculation of flock center
                x_sum += loc_abs[i][0];
                y_sum += loc_abs[i][1];
            }
            // Speed measure
            flock_center[0] = x_sum / ROBOTS_N;
            flock_center[1] = y_sum / ROBOTS_N;
            if (prev_fcenter[0] == -1000 && prev_fcenter[1] == -1000) {
                prev_fcenter[0] = flock_center[0];
                prev_fcenter[1] = flock_center[1];}
            fit_v = sqrtf(powf(flock_center[0]-prev_fcenter[0],2)+powf(flock_center[1]-prev_fcenter[1],2)) / D_max_form;
            // Total metric on this timestep
            fit_step = fit_v / (1 + 1/(ROBOTS_N) * fit_d);
            metric_form += fit_step;
            metric_stepcount++;
            prev_fcenter[0] = flock_center[0];
            prev_fcenter[1] = flock_center[1];
            if (VERBOSE_METRIC)
                {
                    printf("Formation Metric: %f\n", fit_step);
                }
        }
        else if (t > 115.0 && saved == false)
        {
            // Calculate mean of flocking metric
            double meanmet_form;
            meanmet_form = metric_form / metric_stepcount;
            // Print and save to file
            printf("Formation metric: %f\n", meanmet_form);
            FILE *fp;
            fp = fopen("..\\..\\metric_scores\\formation.txt", "w");
            fprintf(fp, "Formation metric score: %f\n", meanmet_form);
            fclose(fp);
            printf("Metric saved to formation.txt\n");
            saved = true;
        }
    }
}

/*
 * Main function.
 */

int main()
{
    init_super();
    // printf("Channel Emitter %d\n", wb_emitter_get_channel(emitter));

    for (;;)
    {
        // Get absolute position of the robot
        get_absolute_position();

        // Receive estimated position of the robot
        get_info();

        // Compute metric
        compute_metric();

        wb_robot_step(time_step);
        t += (float)time_step / 1000;
        // printf("t = %f\n", t);
    }
}
