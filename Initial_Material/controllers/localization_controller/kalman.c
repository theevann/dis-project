
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#include "odometry.h"

#define VERBOSE_POS false

// define constants
const float R[4][4] = {{0.05, 0, 0, 0}, // Input covariance matrix
                       {0, 0.05, 0, 0}, 
                       {0, 0, 0.01, 0},
                       {0, 0, 0, 0.01}};
const float C[2][4] = {{1, 0, 0, 0},
                       {0, 1, 0, 0}};   // Measurement model matrix
const float Q[2][2] = {{1, 0},           // Measurement covariance matrix
                        {0, 1}};
const float I[2][2] = {{1, 0}, // Identity matrix 2x2
                       {0, 1}};
int dt;

// define variables 
float state[4][1];
float prev_state[4][1];
float Cov[4][4];
float prev_Cov[4][4];
float A[4][4];
float B[4][2];
float K[4][2]; // Kalman correction gain
float z;       // Measurement


void init_kalman(float initial_x, float initial_y) {
    dt = wb_robot_get_basic_time_step();

    float A_init[4][4] = {{1, 0, dt, 0}, //Process matrix
                            {0, 1, 0, dt},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1}};
    float B_init[4][2] = {{dt ^ 2 / 2, 0}, // Control matrix
                            {0, dt ^ 2 / 2},
                            {dt, 0},
                            {0, dt}};
    float init_cov[4][4] = {{0.001, 0, 0, 0},
                      {0, 0.001, 0, 0},
                      {0, 0, 0.001, 0},
                      {0, 0, 0, 0.001}};
    float init_state[4][1] = {{initial_x},
                              {initial_y},
                              {0},
                              {0}};
    memcpy(A, A_init, sizeof(A_init));
    memcpy(B, B_init, sizeof(B_init));
    memcpy(prev_Cov, init_cov, sizeof(init_cov));
    memcpy(prev_state, init_state, sizeof(init_state));
}

void update_pos_kalman(position_t* pos, float acc[2], float meas_val[2], bool meas_true) {

    // TO DO: do computations with linear algebra library or with defined functions
    // Be careful with size of state ( use transpose ? )
    // Update state with motion model
    float acc_in[2][1] = {{acc[0]},
                          {acc[1]}};
    state = A*prev_state + B*acc_in;
    Cov = A*prev_Cov*transpose(A) + R*dt;
    
    // Compute correction if the measurement is not extrapolated
    if (meas_true) {
        K = Cov * transpose(C) * inv(C * Cov * transpose(C) + Q);
        state = state + K * ( z - C * state);
        Cov = (I - K * C) * Cov;
    }
}