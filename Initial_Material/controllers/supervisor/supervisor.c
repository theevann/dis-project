/*
 * File:          supervisor_local.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "metrics.h"
#include "../const.h"
#include "../localization/odometry.h"


#define TASK 0 // Tasks: 0 is localization, 1 is flocking, 2 is formation control

#define VERBOSE_SUPPOS false // print position from supervisor

WbNodeRef rob[ROBOTS_N];           // Robot node
WbFieldRef rob_trans[ROBOTS_N];    // Robots translation fields
WbFieldRef rob_rotation[ROBOTS_N]; // Robots rotation fields
WbDeviceTag receiver_rob;              // Receiver for robot positions
WbDeviceTag receiver_pso;               // Receiver for PSO supervisor communication
WbDeviceTag emitter;               // Emitter for PSO supervisor


// Localization
static int time_step;
float loc_abs[ROBOTS_N][3]; // Absolute Location of the robot
float loc_est[ROBOTS_N][3]; // Estimated position

// PSO
static int simulating = 1;

static int end_crit = 0;
static int end_t;
static float start_t;



/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super(void)
{
    wb_robot_init();
    time_step = wb_robot_get_basic_time_step();

    receiver_rob = wb_robot_get_device("receiver0");
    receiver_pso = wb_robot_get_device("receiver1");
    if (receiver_rob == 0 || receiver_pso == 0)
        printf("Missing receiver in supervisor\n");

    emitter = wb_robot_get_device("emitter");   
    wb_emitter_set_channel(emitter, 1);          // PSO supervisor communication is on channel 1

    wb_receiver_enable(receiver_rob, time_step); 
    wb_receiver_set_channel(receiver_rob, 0);
    wb_receiver_enable(receiver_pso, time_step); 
    wb_receiver_set_channel(receiver_pso, 1);       // will not create problems ? receiving its own message ?

    int i;
    char rob_name[7];
    for (i = 0; i < ROBOTS_N; i++)
    {
        sprintf(rob_name, "epuck%d", i); // verify
        printf("Robot name %s \n", rob_name);
        rob[i] = wb_supervisor_node_get_from_def(rob_name);
        rob_trans[i] = wb_supervisor_node_get_field(rob[i], "translation");
        rob_rotation[i] = wb_supervisor_node_get_field(rob[i], "rotation");
    }

    // Find termination crit
    // Find if traj 1 or 2 and if goal reached in flocking or formation
    
    if (TASK == 0 || TASK == 2)
    { 
        if (TRAJECTORY == 1) end_crit = 115;
        if (TRAJECTORY == 2) end_crit = 107;
    }
}


void get_absolute_position(void)
{
    // Get data
    for (int i = 0; i < ROBOTS_N; i++)
    {
        loc_abs[i][0] = wb_supervisor_field_get_sf_vec3f(rob_trans[i])[0];       // X
        loc_abs[i][1] = -wb_supervisor_field_get_sf_vec3f(rob_trans[i])[2];      // Z
        loc_abs[i][2] = wb_supervisor_field_get_sf_rotation(rob_rotation[i])[3]; // THETA

        if (SUPERVISOR_VERBOSE_POSITION)
        {
            printf("(GT) Position of robot%d is x: %f, y: %f, theta: %f\n", i, loc_abs[i][0], loc_abs[i][1], loc_abs[i][2]);
            // printf("(GT) RPosition of robot%d is x: %f, y: %f, theta: %f\n", i, loc_abs[i][0]+2.9, loc_abs[i][1], loc_abs[i][2]+M_PI/2);
        }
    }

    //printf("(GT) Position is x: %f, y: %f, theta: %f\n", loc_abs[0][0], loc_abs[0][1], loc_abs[0][2]);
    //printf("(GT) vx: %g, vy: %g\n", wb_supervisor_node_get_velocity(rob[0])[0], -wb_supervisor_node_get_velocity(rob[0])[2]);
}


void get_info(void)
{
    int rob_nb, count = 0;
    float rob_x, rob_z, rob_theta;
    char *inbuffer;

    while (wb_receiver_get_queue_length(receiver_rob) > 0 && count < ROBOTS_N)
    {
        inbuffer = (char *)wb_receiver_get_data(receiver_rob);
        sscanf(inbuffer, "%d#%f#%f#%f", &rob_nb, &rob_x, &rob_z, &rob_theta);

        loc_est[rob_nb][0] = rob_x;
        loc_est[rob_nb][1] = rob_z;
        loc_est[rob_nb][2] = rob_theta;

        count++;
        wb_receiver_next_packet(receiver_rob);
    }
}


void update_metric(int task)
{
    // Compute the metric for the corresponding task
    if (task == 0)
    {
        update_localization_metric(end_crit, loc_abs, loc_est);
    }
    else if (task == 1)
    {
        update_flocking_metric(end_crit, loc_abs);
    }
    else if (task == 2)
    {
        update_formation_metric(end_crit, loc_abs);
    }
}


int main(void)
{
    init_super();

    while (1)
    {
        if (PSO && !simulating) {

            double *rbuffer;
            float start_t;
            
            while (wb_receiver_get_queue_length(receiver_pso) == 0) {
            wb_robot_step(64);
            }
            rbuffer = (double *)wb_receiver_get_data(receiver_pso);
            end_t = rbuffer[0];
            printf("Metric supervisor received end time %d", end_t);
            start_t = wb_robot_get_time();
            simulating = 1;

            wb_receiver_next_packet(receiver_pso);
        }

        // Get absolute position of the robot
        get_absolute_position();

        // Receive estimated position of the robot
        get_info();

        // Compute metric
        update_metric(TASK);

        wb_robot_step(time_step);

        if (PSO) {
            if (wb_robot_get_time() - start_t >= end_t) {
                // get metric for this round
                double fit;
                fit = get_final_metric(TASK);
                simulating = 0;

                // sending fitness
                double buffer[255];
                buffer[0] = fit;
                wb_emitter_send(emitter,(void *)buffer,sizeof(double));
            }
        }
  }

}
