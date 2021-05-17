#ifndef KALMAN_H
#define KALMAN_H 

void init_kalman(double initial_x, double initial_y);
void update_pos_kalman(position_t* pos, double acc[2], double meas_val[2], bool meas_true);

#endif