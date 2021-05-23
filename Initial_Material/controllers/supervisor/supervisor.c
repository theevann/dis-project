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

#define TASK 0            // Tasks: 0 is localization, 1 is flocking, 2 is formation control
#define VERBOSE_ERR false // print error on each timestep

WbNodeRef rob;           // Robot node
WbFieldRef rob_trans;    // Robots translation fields
WbFieldRef rob_rotation; // Robots rotation fields
WbDeviceTag receiver;    // Single receiver

static int time_step;
static float t = 0.0;
float loc_abs[3]; // Aboslute Location of the robot
float loc_est[3]; // Estimated position
bool saved = false;
float metric = 0.0;
const position_t initial_pos = {-2.9, 0., -M_PI / 2};


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

    char rob_name[7] = "ROBOT1"; // verify
    rob = wb_supervisor_node_get_from_def(rob_name);
    rob_trans = wb_supervisor_node_get_field(rob, "translation");
    rob_rotation = wb_supervisor_node_get_field(rob, "rotation");
}

void get_absolute_position(void)
{
    // Get data
    loc_abs[0] = wb_supervisor_field_get_sf_vec3f(rob_trans)[0];       // X
    loc_abs[1] = -wb_supervisor_field_get_sf_vec3f(rob_trans)[2];      // Z
    loc_abs[2] = wb_supervisor_field_get_sf_rotation(rob_rotation)[3]; // THETA

    printf("(GT) Position is x: %f, y: %f, theta: %f\n", loc_abs[0], loc_abs[1], loc_abs[2]);
    // printf("(GT) vx: %f\n", wb_supervisor_node_get_velocity(rob)[0]);
}

void get_info(void)
{
    char *inbuffer;

    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver);
        sscanf(inbuffer, "%f#%f#%f", &loc_est[0], &loc_est[1], &loc_est[2]);

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
        if (t < 115.0)
        {
            float error;
            error = sqrt(pow((loc_abs[0] - loc_est[0]), 2) + pow((loc_abs[1] - loc_est[1]), 2));
            if (error == error)
            {
                metric += error;
            }
            if (VERBOSE_ERR)
            {
                printf("Error: %f\n", error);
            }
        }
        else if (t > 115.0 && saved == false)
        {
            printf("Localization metric: %f\n", metric);
            FILE *fp;
            fp = fopen("..\\..\\metric_scores\\localization.txt", "w");
            fprintf(fp, "Localization metric score: %f\n", metric);
            fclose(fp);
            printf("Metric saved to localization.txt\n");
            saved = true;
        }
        else
        {
        }
    }
    else if (TASK == 1)
    {
        printf("Flocking metric not implemented yet\n");
    }
    else if (TASK == 2)
    {
        printf("Formation control metric not implemented yet\n");
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
