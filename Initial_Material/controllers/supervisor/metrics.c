#include "metrics.h"
#include "../const.h"


// Metrics and saving
bool saved = false;
long metric_stepcount = 0;

double metric_loc = 0.0;
double metric_flock = 0.0;
double metric_form = 0.0;

static float prev_center[2] = {-1000, -1000};
static const float rel_formation_pos[5][2] = {{0., 0.}, {1.0, 1.0}, {0.5, 0.5}, {0.4, 0.4}, {1.5, 1.5}}; // TO BE DEFINED

static double pso_metric_flock;
static double pso_metric_form;



void update_localization_metric(int end_crit, float loc_abs[ROBOTS_N][3], float loc_est[ROBOTS_N][3])
{
    double time = wb_robot_get_time(); // SECONDS
    
    if (time <= end_crit)
    {
        for (int i = 0; i < ROBOTS_N; i++)
        {
            metric_loc += dist(loc_abs[i], loc_est[i]);

            if (SUPERVISOR_VERBOSE_METRIC)
                printf("Localization error robot%d: %f\n", i, dist(loc_abs[i], loc_est[i]));
        }
    }
    else if (!saved)
    {
        wchar_t* name = L"localization";
        save_metric(name, metric_loc / (double)ROBOTS_N);
    }
}


void update_flocking_metric(int end_crit, float loc_abs[ROBOTS_N][3])
{
    int i, j;
    float flock_center[2] = {0., 0.};
    float dist_to_goal[2];

    // Compute center of flock
    for (i = 0; i < ROBOTS_N; i++)
    {
        flock_center[0] += loc_abs[i][0];
        flock_center[1] += loc_abs[i][1];
    }

    flock_center[0] /= (float)ROBOTS_N;
    flock_center[1] /= (float)ROBOTS_N;

    // Compute distance to goal
    dist_to_goal[0] = fabsf(flock_center[0] - MIGR_X);
    dist_to_goal[1] = fabsf(flock_center[1] - MIGR_Y);

    if (dist_to_goal[0] < .2 && dist_to_goal[1] < .2)  // TODO: define range
        end_crit = 1;

    if (!end_crit)
    {
        float fit_o = 0;
        float fit_d1 = 0;
        float fit_d2 = 0;
        float fit_v = 0;
        double fit_step;
        float dist_diff = 0;

        float N = (float)ROBOTS_N;
        float N_pairs = N * (N-1) / 2;

        if (prev_center[0] == -1000 && prev_center[1] == -1000) // TODO: Looks like an init - do it elsewhere
        {
            prev_center[0] = flock_center[0];
            prev_center[1] = flock_center[1];
        }

        for (i = 0; i < ROBOTS_N; i++)
        {
            // First part of distance measure for each robot
            fit_d1 += dist(loc_abs[i], flock_center);
            
            for (j = i + 1; j < ROBOTS_N; j++)
            {
                // Orientation measure for each pair of robots
                fit_o += fabsf(loc_abs[i][2] - loc_abs[j][2]);

                // Second part of distance measure for each pair of robots
                dist_diff = dist(loc_abs[i], loc_abs[j]);
                fit_d2 += fminf(dist_diff / D_FLOCK, 1 / powf(1 - D_FLOCK + dist_diff, 2));
            }
        }

        fit_d1 = 1. / (1. + fit_d1 / N);
        fit_d2 = fit_d2 / N_pairs;
        fit_o = 1. - fit_o / N_pairs / M_PI;

        // Speed measure
        fit_v = dist(flock_center, prev_center) / D_MAX_FLOCK;
        
        // Total metric on this timestep
        fit_step = fit_o * fit_d1 * fit_d2 * fit_v;
        metric_flock += fit_step;
        metric_stepcount++;

        prev_center[0] = flock_center[0];
        prev_center[1] = flock_center[1];

        if (SUPERVISOR_VERBOSE_METRIC)
            printf("Flocking Metric: %f\n", fit_step);
    }
    else if (!saved && !PSO)
    {
        wchar_t* name = L"flocking";
        save_metric(name, metric_flock / metric_stepcount);
    }
    else if (PSO) 
    {
        pso_metric_flock = metric_flock / metric_stepcount;
        metric_stepcount = 0;
        metric_flock = 0;
        prev_center[0] = -1000;
        prev_center[1] = -1000;
    }
}


void update_formation_metric(int end_crit, float loc_abs[ROBOTS_N][3])
{
    double time = wb_robot_get_time(); // SECONDS

    if (time <= end_crit)
    {
        float formation_center[2] = {0, 0};
        float rob_formation_pos[2];
        float fit_d = 0;
        float fit_v = 0;
        double fit_step;


        // TODO: Not a good idea: reducing the metric - maybe do not count the first iteration
        if (prev_center[0] == -1000 && prev_center[1] == -1000)
        {
            prev_center[0] = formation_center[0];
            prev_center[1] = formation_center[1];
        }

        for (int i = 0; i < ROBOTS_N; i++)
        {
            // Calculate each robot's goal position on this timestep
            rob_formation_pos[0] = loc_abs[LEADER_ID][0] + rel_formation_pos[i][0];
            rob_formation_pos[1] = loc_abs[LEADER_ID][1] + rel_formation_pos[i][1];

            // Distance measure to its goal position for each robot
            fit_d += dist(loc_abs[i], rob_formation_pos);

            // Calculation of formation center
            formation_center[0] += loc_abs[i][0];
            formation_center[1] += loc_abs[i][1];
        }

        fit_d = 1 / (1 + fit_d / (float)ROBOTS_N);
        formation_center[0] /= (float)ROBOTS_N;
        formation_center[1] /= (float)ROBOTS_N;

        // Speed measure
        fit_v = dist(formation_center, prev_center) / D_MAX_FORM;

        // Total metric on this timestep
        fit_step = fit_v * fit_d;
        metric_form += fit_step;
        metric_stepcount++;

        prev_center[0] = formation_center[0];
        prev_center[1] = formation_center[1];

        if (SUPERVISOR_VERBOSE_METRIC)
            printf("Formation Metric: %f\n", fit_step);
    }
    else if (!saved && !PSO)
    {
        wchar_t* name = L"formation";
        save_metric(name, metric_form / metric_stepcount);
    }
    else if (PSO) 
    {
        pso_metric_form = metric_form / metric_stepcount;
        metric_stepcount = 0;
        metric_form = 0;
        prev_center[0] = -1000;
        prev_center[1] = -1000;
    }
}

double get_final_metric(int task) {
    if (task == 1)
        return pso_metric_flock;
    if (task == 2)
        return pso_metric_form;
}



// HELPER FUNCTIONS


float dist(float v1[2], float v2[2])
{
    return sqrtf(powf(v1[0] - v2[0], 2) + powf(v1[1] - v2[1], 2));
}


void save_metric(wchar_t* name, double metric)
{
    char* file_path = (char*)malloc(100 * sizeof(char));
    sprintf(file_path, "..\\..\\metric_scores\\%ls.txt", name);
    
    char* text = (char*)malloc(100 * sizeof(char));
    sprintf(text, "The %ls metric score with %d robots: %f\n", name, ROBOTS_N, metric);

    FILE *fp;
    fp = fopen(file_path, "w");
    fprintf(fp, text);
    fclose(fp);

    printf(text);
    printf("Metric saved to %s\n", file_path);
    saved = true;
}
