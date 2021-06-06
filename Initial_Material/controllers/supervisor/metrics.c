#include "metrics.h"

#define GOAL_FINAL_RANGE .1


// Metrics and saving
bool saved = false;
double start_time = 0.0;
double metric[N_FLOCKING_GROUP];
long metric_stepcount[N_FLOCKING_GROUP];
static float prev_center[N_FLOCKING_GROUP][2];

double d_flock = D_FLOCK;
position_t migrs[N_FLOCKING_GROUP] = {};


int update_localization_metric(float loc_abs[N_ROBOTS][3], float loc_est[N_ROBOTS][3])
{
    double time = wb_robot_get_time() - start_time; // SECONDS
    bool ended;

    //TODO: Take into account start time (eg accelerometer calibration ?)
    if (TRAJECTORY == 1) ended = time > 115;
    if (TRAJECTORY == 2) ended = time > 107;


    if (!ended)
    {
        for (int i = 0; i < N_ROBOTS; i++)
        {
            metric[0] += dist(loc_abs[i], loc_est[i]);

            if (SUPERVISOR_VERBOSE_METRIC)
                printf("Localization error robot%d: %f\n", i, dist(loc_abs[i], loc_est[i]));
        }
    }
    else if (!saved)
    {
        wchar_t* name = L"localization";
        save_metric(name);
    }

    return ended;
}


int update_flocking_metric_group(float loc_abs[N_ROBOTS][3], int group)
{
    int index_shift = group * FLOCK_SIZE;

    double time = wb_robot_get_time() - start_time; // SECONDS
    float flock_center[2] = {0., 0.}, migr[2] = {migrs[group].x, migrs[group].y};
    float *prev_group_center = prev_center[group];
    bool ended;

    // Compute center of flock
    for (int i = index_shift; i < index_shift + FLOCK_SIZE; i++)
    {
        flock_center[0] += loc_abs[i][0];
        flock_center[1] += loc_abs[i][1];
    }

    flock_center[0] /= (float)FLOCK_SIZE;
    flock_center[1] /= (float)FLOCK_SIZE;
 
    if (metric_stepcount[group] == 0)
        memcpy(prev_group_center, flock_center, sizeof(flock_center));

    // Compute distance to goal
    ended = PSO ? (time >= PSO_FIT_T) : (dist(flock_center, migr) < GOAL_FINAL_RANGE);

    // If end criterion is reached, return
    if (ended)
        return 1;

    // Else compute metric
    float fit_o = 0, fit_d1 = 0, fit_d2 = 0, fit_v = 0, fit_step = 0;
    float dist_diff = 0;

    float N = (float)FLOCK_SIZE,
          N_pairs = N * (N-1) / 2;


    for (int i = index_shift; i < index_shift + FLOCK_SIZE; i++)
    {
        // First part of distance measure for each robot
        fit_d1 += dist(loc_abs[i], flock_center);
        
        for (int j = i + 1; j < index_shift + FLOCK_SIZE; j++)
        {
            // Orientation measure for each pair of robots
            fit_o += fabsf(loc_abs[i][2] - loc_abs[j][2]);

            // Second part of distance measure for each pair of robots
            dist_diff = dist(loc_abs[i], loc_abs[j]);
            fit_d2 += fminf(dist_diff / d_flock, 1 / powf(1 - d_flock + dist_diff, 2));
        }
    }

    fit_d1 = 1. / (1. + fit_d1 / N);
    fit_d2 = fit_d2 / N_pairs;
    fit_o = 1. - fit_o / N_pairs / M_PI;

    // Speed measure
    fit_v = dist(flock_center, prev_group_center) / D_MAX_FLOCK;
    
    // Total metric on this timestep
    fit_step = fit_o * fit_d1 * fit_d2 * fit_v;

    metric[group] += fit_step;
    metric_stepcount[group]++;

    memcpy(prev_group_center, flock_center, sizeof(flock_center));

    if (SUPERVISOR_VERBOSE_METRIC)
        printf("Flocking Metric group %d: %f\n", group, fit_step);

    return 0;
}


int update_flocking_metric(float loc_abs[N_ROBOTS][3])
{
    bool all_ended = true;

    for (int i = 0; i < N_FLOCKING_GROUP; i++) 
    {
        all_ended &= update_flocking_metric_group(loc_abs, i);
    }


    if (!all_ended)
        return 1;

    if (!saved && !PSO)
    {
        wchar_t* name = L"flocking";
        save_metric(name);
    }

    return 0;
}


int update_formation_metric_group(float loc_abs[N_ROBOTS][3], int group)
{
    int index_shift = group * FLOCK_SIZE;
    int leader_id = index_shift + LEADER_ID;

    double time = wb_robot_get_time() - start_time; // SECONDS
    float formation_center[2] = {0., 0.}, migr[2] = {migrs[group].x, migrs[group].y};
    float *prev_group_center = prev_center[group];
    bool ended;


    // Compute distance to goal
    ended = PSO ? (time >= PSO_FIT_T) : (dist(loc_abs[leader_id], migr) < GOAL_FINAL_RANGE);

    // If end criterion is reached, return
    if (ended)
        return 1;

    // Else compute metric
    float rob_formation_pos[2];
    float fit_d = 0, fit_v = 0, fit_step = 0;


    for (int i = index_shift; i < index_shift + FLOCK_SIZE; i++)
    {
        // Calculate each robot's goal position on this timestep
        rob_formation_pos[0] = loc_abs[leader_id][0] + FORM_REL_POS[i % FLOCK_SIZE][0];
        rob_formation_pos[1] = loc_abs[leader_id][1] + FORM_REL_POS[i % FLOCK_SIZE][1];

        // Distance measure to its goal position for each robot
        fit_d += dist(loc_abs[i], rob_formation_pos);

        // Calculation of formation center
        formation_center[0] += loc_abs[i][0];
        formation_center[1] += loc_abs[i][1];
    }

    fit_d = 1 / (1 + fit_d / (float)FLOCK_SIZE);
    formation_center[0] /= (float)FLOCK_SIZE;
    formation_center[1] /= (float)FLOCK_SIZE;

    if (metric_stepcount[group] == 0)
        memcpy(prev_group_center, formation_center, sizeof(formation_center));


    // Speed measure
    fit_v = dist(formation_center, prev_group_center) / D_MAX_FORM;

    // Total metric on this timestep
    fit_step = fit_v * fit_d;
    metric[group] += fit_step;
    metric_stepcount[group]++;

    memcpy(prev_group_center, formation_center, sizeof(formation_center));

    if (SUPERVISOR_VERBOSE_METRIC)
        printf("Formation Metric group %d: %f\n", group, fit_step);

    return 0;
}


int update_formation_metric(float loc_abs[N_ROBOTS][3])
{
    bool all_ended = true;

    for (int i = 0; i < N_FLOCKING_GROUP; i++) 
    {
        all_ended &= update_formation_metric_group(loc_abs, i);
    }


    if (!all_ended)
        return 1;

    if (!saved && !PSO)
    {
        wchar_t* name = L"formation";
        save_metric(name);
    }

    return 0;
}


double get_metric() {
    double final_metric = 0, normalizer;

    for (int i = 0; i < N_FLOCKING_GROUP; i++)
    {
        normalizer = TASK == 0 ? N_ROBOTS : metric_stepcount[i];
        final_metric += metric[i] / normalizer;
    }

    // Do some sanity check on current configuration
    if (TASK == 0 && N_FLOCKING_GROUP != 1) printf("Warning: N_FLOCKING_GROUP is not 1");
    
    return final_metric / N_FLOCKING_GROUP;
}


void reset_metric() {
    start_time = wb_robot_get_time();

    for (int i = 0; i < N_FLOCKING_GROUP; i++)
    {
        metric[i] = 0;
        metric_stepcount[i] = 0;
        prev_center[i][0] = 0;
        prev_center[i][1] = 0;
        memcpy(&(migrs[i]), &(MIGR[i]), sizeof(MIGR[i]));
    }
}


void metric_set_dflock(double new_d_flock)
{
    d_flock = new_d_flock;
}


void metric_set_migr(position_t new_migr[N_FLOCKING_GROUP])
{
    for (int i = 0; i < N_FLOCKING_GROUP; i++)
        memcpy(&(migrs[i]), &(new_migr[i]), sizeof(new_migr[i]));
}



// HELPER FUNCTIONS


float dist(float v1[2], float v2[2])
{
    return sqrtf(powf(v1[0] - v2[0], 2) + powf(v1[1] - v2[1], 2));
}


void save_metric(wchar_t* name)
{
    double metric = get_metric();

    char* file_path = (char*)malloc(100 * sizeof(char));
    sprintf(file_path, "..\\..\\metric_scores\\%ls.txt", name);
    
    char* text = (char*)malloc(100 * sizeof(char));
    sprintf(text, "The %ls metric score with %d robots: %f\n", name, N_ROBOTS, metric);

    FILE *fp;
    fp = fopen(file_path, "w");
    fprintf(fp, text);
    fclose(fp);

    printf(text);
    printf("Metric saved to %s\n", file_path);
    saved = true;
}