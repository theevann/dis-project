
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include "odometry.h"

#define STATE_DIM 4
#define MEAS_DIM 2
#define CONTROL_DIM 2
#define VERBOSE_KALMAN_1 true

// define constants
const double R[] = {0.05, 0, 0, 0, // Input covariance matrix
                      0, 0.05, 0, 0, 
                      0, 0, 0.01, 0,
                      0, 0, 0, 0.01};

const double C[] = { 1, 0, 0, 0,
                    0, 1, 0, 0};   // Measurement model matrix

const double Q[] = { 1, 0,           // Measurement covariance matrix
                    0, 1};

const double I[] = { 1, 0, // Identity matrix 2x2
                    0, 1};
double dt;

// define variables 
gsl_vector* state;
gsl_vector* prev_state;
gsl_matrix* Cov;
gsl_matrix* prev_Cov;

gsl_matrix* A;
gsl_matrix* B;


void init_kalman(double initial_x, double initial_y) {
    dt = wb_robot_get_basic_time_step() / 1000.;
    printf("\ndt %f\n", dt);

    double A_init[] = { 1, 0, dt, 0, //Process matrix
                             0, 1,  0, dt,
                             0, 0,  1, 0,
                             0, 0,  0, 1 };

    double B_init[] = {0, 0, // Control matrix
                       0, 0,
                       dt, 0,
                       0, dt};

    double init_cov[] = {0.001, 0, 0, 0,
                         0, 0.001, 0, 0,
                         0, 0, 0.001, 0,
                         0, 0, 0, 0.001};

    // double init_state[] = {initial_x,
    //                       initial_y,
    //                       0,
    //                       0};

    double init_state[] = {0, 0, 0, 0};

    A = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_matrix* A_init_ = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    *A_init_ = gsl_matrix_view_array(A_init, STATE_DIM, STATE_DIM).matrix;
    gsl_matrix_memcpy(A, A_init_);

    B = gsl_matrix_alloc(STATE_DIM, CONTROL_DIM);
    gsl_matrix* B_init_ = gsl_matrix_alloc(STATE_DIM, CONTROL_DIM);
    *B_init_ = gsl_matrix_view_array(B_init, STATE_DIM, CONTROL_DIM).matrix;
    gsl_matrix_memcpy(B, B_init_);

    Cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    prev_Cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    *prev_Cov = gsl_matrix_view_array(init_cov, STATE_DIM, STATE_DIM).matrix;

    state = gsl_vector_alloc(STATE_DIM);
    prev_state = gsl_vector_alloc(STATE_DIM);
    *prev_state = gsl_vector_view_array(init_state, STATE_DIM).vector;

    prev_state = gsl_vector_alloc(STATE_DIM);
    gsl_vector* prev_state_init = gsl_vector_alloc(STATE_DIM);
    *prev_state_init = gsl_vector_view_array(init_state, STATE_DIM).vector;
    gsl_vector_memcpy(prev_state, prev_state_init);


    // printf("\nA = \n");
    // gsl_matrix_fprintf(stdout, A, "%lf");
}


void update_pos_kalman(position_t* pos, double acc[CONTROL_DIM], double meas_val[MEAS_DIM], bool meas_true) {
    // TO DO: do computations with linear algebra library or with defined functions
    // Be careful with size of state ( use transpose ? )
    // Update state with motion model
    gsl_matrix* K = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);
    gsl_vector* z = gsl_vector_alloc(MEAS_DIM);

    gsl_vector* input = gsl_vector_alloc(CONTROL_DIM);
    double input_data[2] = {acc[1], -acc[0]};
    *input = gsl_vector_view_array(input_data, CONTROL_DIM).vector;
    
    // printf("ACC CURR %g, %g, %g \n\n", acc[0], acc[1], acc[2]);
    // printf("\nA = \n");
    // gsl_matrix_fprintf(stdout, A, "%lf");
    // printf("\nB = \n");
    // gsl_matrix_fprintf(stdout, B, "%lf");
    // printf("\ninput = \n");
    // gsl_vector_fprintf(stdout, input, "%lf");
    // printf("\nstate before = \n");
    // gsl_vector_fprintf(stdout, state, "%lf");

    gsl_blas_dgemv(CblasNoTrans, 1.0, A, prev_state, 0.0, state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, B, input, 1.0, state);

    // state = A*prev_state + B*input;
    // Cov = A*prev_Cov*transpose(A) + R*dt;
    
    // // Compute correction if the measurement is not extrapolated
    // if (meas_true) {
    //     K = Cov * transpose(C) * inv(C * Cov * transpose(C) + Q);
    //     state = state + K * ( meas_val - C * state);
    //     Cov = (I - K * C) * Cov;
    // }

    // prev_Cov = Cov
    gsl_vector_memcpy(prev_state, state);

    pos->x = state->data[0];
    pos->y = state->data[1];

    if (VERBOSE_KALMAN_1)
		printf("Kalman basic with acceleration : %g %g %g\n", pos->x , pos->y , RAD2DEG(pos->heading));
}