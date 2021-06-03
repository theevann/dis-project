#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>

#include "../const.h"


void update_localization_metric(int end_crit, float loc_abs[ROBOTS_N][3], float loc_est[ROBOTS_N][3]);
void update_flocking_metric(int end_crit, float loc_abs[ROBOTS_N][3]);
void update_formation_metric(int end_crit, float loc_abs[ROBOTS_N][3]);
float dist(float v1[2], float v2[2]);
void save_metric(wchar_t* name, double metric);
double get_final_metric(int task);
