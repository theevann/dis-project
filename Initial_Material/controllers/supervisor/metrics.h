#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>

#include "../const.h"


int update_localization_metric(float loc_abs[N_ROBOTS][3], float loc_est[N_ROBOTS][3]);
int update_flocking_metric_group(float loc_abs[N_ROBOTS][3], int group);
int update_formation_metric_group(float loc_abs[N_ROBOTS][3], int group);
int update_flocking_metric(float loc_abs[N_ROBOTS][3]);
int update_formation_metric(float loc_abs[N_ROBOTS][3]);
double get_metric();
void reset_metric();
void metric_set_dflock(double new_d_flock);
void metric_set_migr(position_t new_migr[N_FLOCKING_GROUP]);
void save_metric(wchar_t* name);
float dist(float v1[2], float v2[2]);
