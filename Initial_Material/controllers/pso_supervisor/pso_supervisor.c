#include <stdio.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#include "../const.h" // TODO: put definitions in const.h file
#include "../supervisor/metrics.h"
#include "../pso/pso.h"


/* Neighborhood types */
#define STANDARD -1
#define RAND_NB 0
#define NCLOSE_NB 1
#define FIXEDRAD_NB 2
#define NEIGHBORHOOD STANDARD // TODO: do or delete


#define FINALRUNS 5
#define RADIUS 0.8


WbNodeRef robots[N_ROBOTS];
WbFieldRef rob_trans[N_ROBOTS];    // Robots translation fields
WbFieldRef rob_rotation[N_ROBOTS]; // Robots rotation fields

WbDeviceTag emitter;
WbDeviceTag receiver;
float loc_abs[N_ROBOTS][3];
double *loc[N_ROBOTS];
double *rot[N_ROBOTS];

static int time_step;


void init_super();
void get_absolute_position();
int update_metric(int task);
char valid_locs(int rob_id, int *j);
void random_pos();
double fitness(double[DATASIZE]);


/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super()
{
    wb_robot_init();
    time_step = wb_robot_get_basic_time_step();


    receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, time_step);

    emitter = wb_robot_get_device("emitter");   
    

    if (N_ROBOTS > 10)
        printf("==== ERROR ====\n More than 10 ROBOTS");

    char robot_name[7];
    for (int i = 0; i < N_ROBOTS; i++)
    {
        sprintf(robot_name, "epuck%d", i);
        // printf("Robot name %s \n", robot_name);
        robots[i] = wb_supervisor_node_get_from_def(robot_name);
        rob_trans[i] = wb_supervisor_node_get_field(robots[i], "translation");
        rob_rotation[i] = wb_supervisor_node_get_field(robots[i], "rotation");

        loc[i] = wb_supervisor_field_get_sf_vec3f(rob_trans[i]);
        rot[i] = wb_supervisor_field_get_sf_rotation(rob_rotation[i]);
    }
}


void get_absolute_position()
{
    for (int i = 0; i < N_ROBOTS; i++)
    {
        loc_abs[i][0] = wb_supervisor_field_get_sf_vec3f(rob_trans[i])[0];       // X
        loc_abs[i][1] = -wb_supervisor_field_get_sf_vec3f(rob_trans[i])[2];      // Z
        loc_abs[i][2] = wb_supervisor_field_get_sf_rotation(rob_rotation[i])[3]; // THETA
    }
}


int update_metric(int task)
{
    // Compute the metric for the corresponding task
    // TO DO: add localization metric to merge supervisors ?
    if (task == 1)
    {
        return update_flocking_metric(loc_abs);
    }
    else if (task == 2)
    {
        return update_formation_metric(loc_abs);
    }
    
    printf("YOU SHOULD NOT BE HERE");
    return 0;
}


/* MAIN - Distribute and test controllers */
int main()
{
    if (PSO) {
        double weights[DATASIZE];    // Optimized result
        double buffer[255]; // Buffer for emitter
        int i, j;        // Counter variables

        /* Initialisation */
        init_super();
        wb_robot_step(256);


        double fit;         // Fitness of the current FINALRUN
        double endfit;      // Best fitness over 10 runs
        double f;           // Evaluated fitness (modified by fitness() )
        double bestfit, bestw[DATASIZE];

        /* Evolve controllers */
        endfit = 0.0;
        bestfit = 0.0;

        printf("Started PSO Optimization \n");
        // Do 10 runs and send the best controller found to the robot
        for (j = 0; j < PSO_N_RESTART; j++)
        {
            printf("  New PSO Optimisation started (%d / %d)\n", j+1, PSO_N_RESTART);

            // Get result of optimization
            pso(SWARMSIZE, NB, LWEIGHT, NBWEIGHT, VMAX, MININIT, MAXINIT, PSO_ITS, DATASIZE, &fitness, weights);


            fit = 0.0;
            // Run FINALRUN tests and calculate average
            for (i = 0; i < FINALRUNS; i++)
            {
                f = fitness(weights);
                fit += f;
            }
            fit /= FINALRUNS;


            // Check for new best fitness
            if (fit > bestfit)
            {
                bestfit = fit;
                copyParticle(bestw, weights);
            }

            printf("Performance of the best solution: %.4f\n", fit);
            endfit += fit / 10; // average over the 10 runs
        }

        printf("~~~~~~~~ Optimization finished.\n");
        printf("Best performance: %.3f\n", bestfit);
        printf("Average performance: %.3f\n", endfit);

        /* Send best controller to robots */
        printf("BEST WEIGHTS\n");
        buffer[0] = 2; // Send pso weights
        buffer[1] = -1; // To all

        for (j = 0; j < DATASIZE; j++)
        {
            buffer[j+2] = bestw[j];
            printf("%f ", bestw[j]);
        }
        printf("\n");
        
        wb_emitter_send(emitter, (void *)buffer, (DATASIZE + 2) * sizeof(double));

        return 0;
    }
}


// Makes sure no robots are overlapping        // TODO: add verification that there is no obstacle there
char valid_locs(int rob_id, int *j)
{
    for (int i = 0; i < N_ROBOTS; i++)
    {
        if (rob_id == i)
            continue;

        if (sqrt(pow(loc[i][0] - loc[rob_id][0], 2) + pow(loc[i][2] - loc[rob_id][2], 2)) <
            (2 * ROB_RADIUS + 0.01))
        {
            (*j)++;
            return 0;
        }
    }

    return 1;
}


// Randomly position specified robot
void random_pos()
{
    //printf("Setting random position for %d\n",rob_id);
    float leader_pos[2];
    leader_pos[0] = randIn(-ARENA_SIZE_X / 2.0, ARENA_SIZE_X / 2.0);
    leader_pos[1] = randIn(-ARENA_SIZE_Y / 2.0, ARENA_SIZE_Y / 2.0);

    int i;
    for (i = 0; i < N_ROBOTS; i++)
    {
        rot[i][0] = 0.0;
        rot[i][1] = 1.0;
        rot[i][2] = 0.0;
        rot[i][3] = 2.0 * 3.14159 * randIn(0, 1);

        int j = 0;
        do
        {
            // disposition 2 by 2 0.5m apart
            loc[i][0] = leader_pos[0] + (i % 2) * 0.2 + j * 0.03; // add small values to find new places without robots and obstacles
            loc[i][1] = 0.0;
            loc[i][2] = leader_pos[1] + (i / 2) * 0.2 + j * 0.03; // division will result in an integer
        } while (!valid_locs(i, &j));

        
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robots[i], "rotation"), rot[i]);
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robots[i], "translation"), loc[i]);
        wb_supervisor_node_reset_physics(robots[i]); // Set velocities and acceleration to 0 - Useful in case the robots fell
    }
}


void initial_pos()
{
    for (int i = 0; i < N_ROBOTS; i++)
    {
        rot[i][0] = 0.0;
        rot[i][1] = 1.0;
        rot[i][2] = 0.0;
        rot[i][3] = INIT_POS[i].heading;

        loc[i][0] = INIT_POS[i].x;
        loc[i][1] = 0.0;
        loc[i][2] = -INIT_POS[i].y;
        
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robots[i], "rotation"), rot[i]);
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robots[i], "translation"), loc[i]);
        wb_supervisor_node_reset_physics(robots[i]); // Set velocities and acceleration to 0 - Useful in case the robots fell
    } 
}


void random_goal(position_t new_migr[N_FLOCKING_GROUP])
{
    for (int i = 0; i < N_FLOCKING_GROUP; i++)
    {
        new_migr[i].x = randIn(-ARENA_SIZE_X / 2.0, ARENA_SIZE_X / 2.0);
        new_migr[i].y = randIn(-ARENA_SIZE_Y / 2.0, ARENA_SIZE_Y / 2.0);
    }
}


double fitness(double weights[DATASIZE])
{
    double buffer[255];
    position_t new_migr[N_FLOCKING_GROUP];

    /* Position all robots and goal */

    if (PSO_RAND_MIGRPOS)  // [only in world 1]
    {
        random_pos();  // Position roboty around one random spot // TODO: Handle only one group (ok for world 1)
        random_goal(new_migr); // Set migratory goal randomly  [only in world 1] // TODO: Handle only one group (ok for world 1)
    }
    else  // [in world 2]
        initial_pos(); // Put robot back to their initial position

    /*
    Messages: always sending double[],
    - First value is message_type:
     - 0 for robot sending localization,
     - 1 for robot sending ping,
     - 2 for supervisor sending pso weights,
     - 3 for supervisor sending position,
     - 4 for supervisor updating MIGR
    - Second value is recipient id, using -1 = all
    */

    /* Send pso weights to robots */
    buffer[0] = 2;
    buffer[1] = -1;

    for (int i = 0; i < DATASIZE; i++)
        buffer[i+2] = weights[i];

    wb_emitter_send(emitter, (void *)buffer, (2 + DATASIZE) * sizeof(double));


    /* Send MIGR to robots */
    // Note: If we were to randomise goal for multi group, we need changes here.
    if (PSO_RAND_MIGRPOS) { 
        buffer[0] = 4;
        buffer[1] = -1;
        buffer[2] = new_migr[0].x; // For now, just send first new goal
        buffer[3] = new_migr[0].y;

        wb_emitter_send(emitter, (void *)buffer, (2 + 2) * sizeof(double));
    }


    /* Send their new position to robots */

    for (int i = 0; i < N_ROBOTS; i++)
    {
        buffer[0] = 3;
        buffer[1] = i;

        get_absolute_position();
        buffer[2] = loc_abs[i][0];
        buffer[3] = loc_abs[i][1];
        buffer[4] = loc_abs[i][2];
        
        wb_emitter_send(emitter, (void *)buffer, (2 + 3) * sizeof(double));
    }


    /* Compute the metric */

    reset_metric();
    if (TASK == 1)
        metric_set_dflock(weights[3]); // A bit ugly (should use global indices)
    if (PSO_RAND_MIGRPOS)
        metric_set_migr(new_migr);

    bool running = true;
    while (running)
    {
        get_absolute_position();
        running = (bool)update_metric(TASK);
        wb_robot_step(time_step);
    }

    return get_metric();
}