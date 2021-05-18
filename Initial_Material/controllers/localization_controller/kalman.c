
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
// double R_data[] = {0.05, 0, 0, 0, // Input covariance matrix
//                    0, 0.05, 0, 0,
//                    0, 0, 0.01, 0,
//                    0, 0, 0, 0.01};
double R_data[] = {0.05, 0, 0, 0, // Input covariance matrix
                   0, 0.05, 0, 0,
                   0, 0, 0.05, 0,
                   0, 0, 0, 0.05};

double C_data[] = {1, 0, 0, 0, // Measurement model matrix
                   0, 1, 0, 0};

// double Q_data[] = {1, 0, // Measurement covariance matrix
//                    0, 1};
double Q_data[] = {.01, 0, // Measurement covariance matrix
                   0, .01};

double dt;

// define variables 
gsl_vector* state;
gsl_matrix* Cov;
gsl_matrix* prev_Cov;

gsl_matrix* A;
gsl_matrix* B;


gsl_matrix* array2matrix(double* array, int size_1, int size_2);
gsl_matrix *invert_matrix(gsl_matrix *matrix, int size);


void init_kalman(double initial_x, double initial_y) {
    dt = wb_robot_get_basic_time_step() / 1000.;
    printf("\ndt %f\n", dt);

    double A_init[] = {1, 0, dt, 0, //Process matrix
                       0, 1, 0, dt,
                       0, 0, 1, 0,
                       0, 0, 0, 1};

    double B_init[] = {0, 0, // Control matrix
                       0, 0,
                       dt, 0,
                       0, dt};

    double init_cov[] = {0.001, 0, 0, 0,
                         0, 0.001, 0, 0,
                         0, 0, 0.001, 0,
                         0, 0, 0, 0.001};

    // double init_state[] = {initial_x, initial_y, 0, 0};
    double init_state[] = {0, 0, 0, 0};

    A = array2matrix(A_init, STATE_DIM, STATE_DIM);
    B = array2matrix(B_init, STATE_DIM, CONTROL_DIM);
    Cov = array2matrix(init_cov, STATE_DIM, STATE_DIM);

    state = gsl_vector_alloc(STATE_DIM);
    gsl_vector* state_init = gsl_vector_alloc(STATE_DIM);
    *state_init = gsl_vector_view_array(init_state, STATE_DIM).vector;
    gsl_vector_memcpy(state, state_init);
}


void update_pos_kalman(position_t* pos, double acc[CONTROL_DIM], double meas_val[MEAS_DIM], bool meas_true)
{
    gsl_vector* z = gsl_vector_alloc(MEAS_DIM);

    double input_data[2] = {acc[1], -acc[0]};
    gsl_vector* input = gsl_vector_alloc(CONTROL_DIM);
    *input = gsl_vector_view_array(input_data, CONTROL_DIM).vector;
    gsl_vector* meas = gsl_vector_alloc(MEAS_DIM);
    *meas = gsl_vector_view_array(meas_val, MEAS_DIM).vector;
    
    // printf("\n\nACC CURR %g, %g, %g \n\n", acc[0], acc[1], acc[2]);
    // printf("\nA = \n");gsl_matrix_fprintf(stdout, A, "%lf");
    // printf("\nB = \n");gsl_matrix_fprintf(stdout, B, "%lf");
    // printf("\ninput = \n");gsl_vector_fprintf(stdout, input, "%lf");
    // printf("\nstate before = \n");gsl_vector_fprintf(stdout, state, "%lf");

    /*  1. Update state with motion model   */

    /*  1.1   state = A * prev_state + B * input   */
    gsl_vector* temp_state = gsl_vector_alloc(STATE_DIM);
    gsl_blas_dgemv(CblasNoTrans, 1.0, A, state, 0.0, temp_state);
    gsl_vector_memcpy(state, temp_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, B, input, 1.0, state);

    /* 1.2   Cov = A * prev_Cov * transpose(A) + R * dt  */
    gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, Cov, 0.0, temp_cov);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, temp_cov, A, 0.0, Cov);
    gsl_matrix* R = array2matrix(R_data, STATE_DIM, STATE_DIM);
    gsl_matrix_scale(R, dt);
    gsl_matrix_add(Cov, R);
    // printf("\nCOV = \n");gsl_matrix_fprintf(stdout, Cov, "%lf");
    
    /*  2. Compute correction if we have a GPS update  */
    if (meas_true) {
        gsl_matrix_view Q = gsl_matrix_view_array(Q_data, MEAS_DIM, MEAS_DIM);
        gsl_matrix_view C = gsl_matrix_view_array(C_data, MEAS_DIM, STATE_DIM);

        /*  2.1   K = Cov * transpose(C) * inv(C * Cov * transpose(C) + Q)  */
        gsl_matrix* CovCT = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);  // Cov * transpose(C)
        gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Cov, &C.matrix, 0.0, CovCT);
        gsl_matrix* CCovCT = gsl_matrix_alloc(MEAS_DIM, MEAS_DIM);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &C.matrix, CovCT, 0.0, CCovCT);
        gsl_matrix_add(CCovCT, &Q.matrix);
        // printf("\nCCovCT = \n");gsl_matrix_fprintf(stdout, CCovCT, "%lf");
        gsl_matrix* inv_CCovCT = invert_matrix(CCovCT, MEAS_DIM);
        // printf("\ninv_CCovCT = \n");gsl_matrix_fprintf(stdout, inv_CCovCT, "%lf");
        gsl_matrix* K = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, CovCT, inv_CCovCT, 0.0, K);
        // printf("\nK = \n");gsl_matrix_fprintf(stdout, K, "%lf");

        /*  2.2   state = state + K * ( meas_val - C * state)  */
        // printf("\nmeas = \n");gsl_vector_fprintf(stdout, meas, "%lf");
        // printf("\nstate = \n");gsl_vector_fprintf(stdout, state, "%lf");
        gsl_vector* C_state = gsl_vector_alloc(MEAS_DIM);
        gsl_blas_dgemv(CblasNoTrans, 1.0, &C.matrix, state, 0.0, C_state);
        gsl_vector* meas_Cstate = gsl_vector_alloc(MEAS_DIM);
        gsl_vector_memcpy(meas_Cstate, meas);
        gsl_vector_sub(meas_Cstate, C_state);
        gsl_blas_dgemv(CblasNoTrans, 1.0, K, meas_Cstate, 1.0, state);
        // printf("\nstate = \n");gsl_vector_fprintf(stdout, state, "%lf");


        /*  2.3   Cov = (I - K * C) * Cov  */
        // printf("\nCOV = \n");gsl_matrix_fprintf(stdout, Cov, "%lf");
        gsl_matrix* I_KC = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
        gsl_matrix_set_identity(I_KC);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, &C.matrix, 1.0, I_KC);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, I_KC, Cov, 0.0, temp_cov);
        gsl_matrix_memcpy(Cov, temp_cov);
        // printf("\nCOV = \n");gsl_matrix_fprintf(stdout, Cov, "%lf");

    }

    pos->x = state->data[0];
    pos->y = state->data[1];

    if (VERBOSE_KALMAN_1)
		printf("Kalman basic with acceleration : %g %g %g\n", pos->x , pos->y , RAD2DEG(pos->heading));
}

gsl_matrix *array2matrix(double *array, int size_1, int size_2)
{
    gsl_matrix *mat = gsl_matrix_alloc(size_1, size_2);
    gsl_matrix *mat_view = gsl_matrix_alloc(size_1, size_2);
    *mat_view = gsl_matrix_view_array(array, size_1, size_2).matrix;
    gsl_matrix_memcpy(mat, mat_view);
    return mat;
}

gsl_matrix *invert_matrix(gsl_matrix *matrix, int size)
{
    int s;
    gsl_matrix *inv = gsl_matrix_alloc(size, size);
    gsl_permutation *p = gsl_permutation_alloc(size);

    gsl_linalg_LU_decomp(matrix, p, &s);
    gsl_linalg_LU_invert(matrix, p, inv);
    gsl_permutation_free(p);

    return inv;
}

// gsl_matrix* array2ematrix(gsl_matrix* mat, const double* array, int size_1, int size_2) {
//     gsl_matrix* mat_view = gsl_matrix_alloc(size_1, size_2);
//     *mat_view = gsl_matrix_view_array(array, size_1, size_2).matrix;
//     gsl_matrix_memcpy(mat, mat_view);
//     return mat;