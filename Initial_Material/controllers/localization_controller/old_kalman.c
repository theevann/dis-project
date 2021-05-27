
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include "odometry.h"
#include "kalman.h"

#define STATE_DIM 4
#define MEAS_DIM 2
#define CONTROL_DIM 2
#define VERBOSE_KALMAN_1 true


double dt;

/*  Define kalman variables  */

gsl_vector* state;
gsl_matrix* cov;
gsl_matrix* A;
gsl_matrix* B;
gsl_matrix* R;

gsl_matrix_view C;
gsl_matrix_view Q;
double C_data[MEAS_DIM * STATE_DIM];
double Q_data[MEAS_DIM * MEAS_DIM];



void init_kalman_wheelvelocity(const position_t* initial_pos)
{
    init_kalman(initial_pos);

    // Control matrix
    double B_data[] = {0, 0,
                       0, 0,
                       dt, 0,
                       0, dt};

    // Measurement model matrix
    double C_data_acc[] = {1, 0, 0, 0,
                           0, 1, 0, 0};

    // Measurement covariance matrix
    double Q_data_acc[] = {.01, 0,
                           0, .01};

    memcpy(C_data, C_data_acc, sizeof(C_data_acc));
    memcpy(Q_data, Q_data_acc, sizeof(Q_data_acc));

    B = array2matrix(B_data, STATE_DIM, CONTROL_DIM);
}


void init_kalman_acc(const position_t* initial_pos)
{
    init_kalman(initial_pos);

    // Control matrix
    double B_data[] = {0, 0,
                       0, 0,
                       dt, 0,
                       0, dt};

    // Measurement model matrix
    double C_data_acc[] = {1, 0, 0, 0,
                           0, 1, 0, 0};

    // Measurement covariance matrix
    double Q_data_acc[] = {.01, 0,
                           0, .01};

    memcpy(C_data, C_data_acc, sizeof(C_data_acc));
    memcpy(Q_data, Q_data_acc, sizeof(Q_data_acc));

    B = array2matrix(B_data, STATE_DIM, CONTROL_DIM);
}


void init_kalman(const position_t* initial_pos) {
    dt = wb_robot_get_basic_time_step() / 1000.;

    // Process matrix
    double A_data[] = {1, 0, dt, 0,
                       0, 1, 0, dt,
                       0, 0, 1, 0,
                       0, 0, 0, 1};
    
    // Input covariance matrix
    double R_data[] = {0.05, 0, 0, 0,
                       0, 0.05, 0, 0,
                       0, 0, 0.05, 0,
                       0, 0, 0, 0.05};
    // double R_data[] = {0.05, 0, 0, 0,
    //                    0, 0.05, 0, 0,
    //                    0, 0, 0.01, 0,
    //                    0, 0, 0, 0.01};

    double state_init[] = {initial_pos->x, initial_pos->y, 0, 0};
    double cov_init[] = {0.001, 0, 0, 0,
                         0, 0.001, 0, 0,
                         0, 0, 0.001, 0,
                         0, 0, 0, 0.001};

    A = array2matrix(A_data, STATE_DIM, STATE_DIM);
    R = array2matrix(R_data, STATE_DIM, STATE_DIM);
    gsl_matrix_scale(R, dt);

    state = array2vector(state_init, STATE_DIM);
    cov = array2matrix(cov_init, STATE_DIM, STATE_DIM);
}


void kalman_prediction_step_acc(position_t* pos, double acc[CONTROL_DIM])
{
    double input_data[2] = {acc[1], -acc[0]};
    gsl_vector* input = gsl_vector_alloc(CONTROL_DIM);
    *input = gsl_vector_view_array(input_data, CONTROL_DIM).vector;

    kalman_prediction_step(pos, input);
}


/*  1. Update state with motion model  */
void kalman_prediction_step(position_t* pos, gsl_vector* input)
{
    // printf("\ninput = \n");gsl_vector_fprintf(stdout, input, "%lf");

    /*  1.1   state = A * prev_state + B * input  */
    gsl_vector* temp_state = gsl_vector_alloc(STATE_DIM);
    gsl_blas_dgemv(CblasNoTrans, 1.0, A, state, 0.0, temp_state);
    gsl_vector_memcpy(state, temp_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, B, input, 1.0, state);


    /*  1.2   cov = A * prev_cov * transpose(A) + R * dt  */
    gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, cov, 0.0, temp_cov);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, temp_cov, A, 0.0, cov);
    gsl_matrix_add(cov, R);


    pos->x = state->data[0];
    pos->y = state->data[1];
}


/*  2. Compute correction if we have a GPS update  */
void kalman_correction_step(position_t* pos, double meas_val[MEAS_DIM])
{
    gsl_vector* meas = gsl_vector_alloc(MEAS_DIM);
    *meas = gsl_vector_view_array(meas_val, MEAS_DIM).vector;

    Q = gsl_matrix_view_array(Q_data, MEAS_DIM, MEAS_DIM);
    C = gsl_matrix_view_array(C_data, MEAS_DIM, STATE_DIM);
    

    /*  2.1   K = cov * transpose(C) * inv(C * cov * transpose(C) + Q)  */
    gsl_matrix* covCT = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);  // cov * transpose(C)
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, cov, &C.matrix, 0.0, covCT);
    gsl_matrix* CcovCT = gsl_matrix_alloc(MEAS_DIM, MEAS_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &C.matrix, covCT, 0.0, CcovCT);
    gsl_matrix_add(CcovCT, &Q.matrix);
    gsl_matrix* inv_CcovCT = invert_matrix(CcovCT, MEAS_DIM);
    gsl_matrix* K = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, covCT, inv_CcovCT, 0.0, K);


    /*  2.2   state = state + K * ( meas_val - C * state)  */
    gsl_vector* C_state = gsl_vector_alloc(MEAS_DIM);
    gsl_blas_dgemv(CblasNoTrans, 1.0, &C.matrix, state, 0.0, C_state);
    gsl_vector* meas_Cstate = gsl_vector_alloc(MEAS_DIM);
    gsl_vector_memcpy(meas_Cstate, meas);
    gsl_vector_sub(meas_Cstate, C_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, K, meas_Cstate, 1.0, state);


    /*  2.3   cov = (I - K * C) * cov  */
    gsl_matrix* I_KC = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_matrix_set_identity(I_KC);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, &C.matrix, 1.0, I_KC);
    gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, I_KC, cov, 0.0, temp_cov);
    gsl_matrix_memcpy(cov, temp_cov);

    pos->x = state->data[0];
    pos->y = state->data[1];
}


void update_pos_kalman(position_t* pos, double acc[CONTROL_DIM], double meas_val[MEAS_DIM], bool meas_true)
{
    kalman_prediction_step_acc(pos, acc);
    if (meas_true) kalman_correction_step(pos, meas_val);
    
    if (VERBOSE_KALMAN_1)
		printf("Kalman basic with acceleration : %g %g %g\n", pos->x , pos->y , RAD2DEG(pos->heading));
}



// void update_pos_kalman(position_t* pos, double acc[CONTROL_DIM], double meas_val[MEAS_DIM], bool meas_true)
// {


//     double input_data[2] = {acc[1], -acc[0]};
//     gsl_vector* input = gsl_vector_alloc(CONTROL_DIM);
//     *input = gsl_vector_view_array(input_data, CONTROL_DIM).vector;
//     gsl_vector* meas = gsl_vector_alloc(MEAS_DIM);
//     *meas = gsl_vector_view_array(meas_val, MEAS_DIM).vector;
    
//     // printf("\n\nACC CURR %g, %g, %g \n\n", acc[0], acc[1], acc[2]);
//     // printf("\nQ = \n");gsl_matrix_fprintf(stdout, &Q.matrix, "%lf");
//     // printf("\nC = \n");gsl_matrix_fprintf(stdout, &C.matrix, "%lf");
//     // printf("\ninput = \n");gsl_vector_fprintf(stdout, input, "%lf");
//     // printf("\nstate before = \n");gsl_vector_fprintf(stdout, state, "%lf");

//     /*  1. Update state with motion model   */

//     /*  1.1   state = A * prev_state + B * input   */
//     gsl_vector* temp_state = gsl_vector_alloc(STATE_DIM);
//     gsl_blas_dgemv(CblasNoTrans, 1.0, A, state, 0.0, temp_state);
//     gsl_vector_memcpy(state, temp_state);
//     gsl_blas_dgemv(CblasNoTrans, 1.0, B, input, 1.0, state);

//     /* 1.2   cov = A * prev_cov * transpose(A) + R * dt  */
//     gsl_matrix* temp_cov = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
//     gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, cov, 0.0, temp_cov);
//     gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, temp_cov, A, 0.0, cov);
//     gsl_matrix_add(cov, R);
//     // printf("\ncov = \n");gsl_matrix_fprintf(stdout, cov, "%lf");
    

//     /*  2. Compute correction if we have a GPS update  */
//     if (meas_true) {
//         Q = gsl_matrix_view_array(Q_data, MEAS_DIM, MEAS_DIM);
//         C = gsl_matrix_view_array(C_data, MEAS_DIM, STATE_DIM);
        
//         /*  2.1   K = cov * transpose(C) * inv(C * cov * transpose(C) + Q)  */
//         gsl_matrix* covCT = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);  // cov * transpose(C)
//         gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, cov, &C.matrix, 0.0, covCT);
//         gsl_matrix* CcovCT = gsl_matrix_alloc(MEAS_DIM, MEAS_DIM);
//         gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &C.matrix, covCT, 0.0, CcovCT);
//         gsl_matrix_add(CcovCT, &Q.matrix);
//         // printf("\nCcovCT = \n");gsl_matrix_fprintf(stdout, CcovCT, "%lf");
//         gsl_matrix* inv_CcovCT = invert_matrix(CcovCT, MEAS_DIM);
//         // printf("\ninv_CcovCT = \n");gsl_matrix_fprintf(stdout, inv_CcovCT, "%lf");
//         gsl_matrix* K = gsl_matrix_alloc(STATE_DIM, MEAS_DIM);
//         gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, covCT, inv_CcovCT, 0.0, K);
//         // printf("\nK = \n");gsl_matrix_fprintf(stdout, K, "%lf");

//         /*  2.2   state = state + K * ( meas_val - C * state)  */
//         // printf("\nmeas = \n");gsl_vector_fprintf(stdout, meas, "%lf");
//         // printf("\nstate = \n");gsl_vector_fprintf(stdout, state, "%lf");
//         gsl_vector* C_state = gsl_vector_alloc(MEAS_DIM);
//         gsl_blas_dgemv(CblasNoTrans, 1.0, &C.matrix, state, 0.0, C_state);
//         gsl_vector* meas_Cstate = gsl_vector_alloc(MEAS_DIM);
//         gsl_vector_memcpy(meas_Cstate, meas);
//         gsl_vector_sub(meas_Cstate, C_state);
//         gsl_blas_dgemv(CblasNoTrans, 1.0, K, meas_Cstate, 1.0, state);
//         // printf("\nstate = \n");gsl_vector_fprintf(stdout, state, "%lf");


//         /*  2.3   cov = (I - K * C) * cov  */
//         // printf("\ncov = \n");gsl_matrix_fprintf(stdout, cov, "%lf");
//         gsl_matrix* I_KC = gsl_matrix_alloc(STATE_DIM, STATE_DIM);
//         gsl_matrix_set_identity(I_KC);
//         gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, &C.matrix, 1.0, I_KC);
//         gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, I_KC, cov, 0.0, temp_cov);
//         gsl_matrix_memcpy(cov, temp_cov);
//         // printf("\ncov = \n");gsl_matrix_fprintf(stdout, cov, "%lf");
//     }

//     pos->x = state->data[0];
//     pos->y = state->data[1];

//     if (VERBOSE_KALMAN_1)
// 		printf("Kalman basic with acceleration : %g %g %g\n", pos->x , pos->y , RAD2DEG(pos->heading));
// }


gsl_vector *array2vector(double *array, int size)
{
    gsl_vector *vec = gsl_vector_alloc(size);
    gsl_vector *vec_view = gsl_vector_alloc(size);
    *vec_view = gsl_vector_view_array(array, size).vector;
    gsl_vector_memcpy(vec, vec_view);
    return vec;
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
// }