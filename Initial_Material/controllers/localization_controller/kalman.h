#ifndef KALMAN_H
#define KALMAN_H 

void init_kalman(float initial_x, float initial_y);
void update_pos_kalman(position_t* pos, float acc[2], float meas_val[2], bool meas_true);

#endif