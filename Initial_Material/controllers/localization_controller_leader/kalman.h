#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#ifndef KALMAN_H
#define KALMAN_H 

gsl_matrix* array2matrix(double* array, int size_1, int size_2);
gsl_matrix* invert_matrix(gsl_matrix *matrix, int size);
void init_kalman(const position_t* initial_pos);
void update_pos_kalman(position_t* pos, double acc[2], double meas_val[2], bool meas_true);

#endif