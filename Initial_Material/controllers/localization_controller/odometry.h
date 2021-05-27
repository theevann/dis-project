#ifndef ODOMETRY_H
#define ODOMETRY_H 

#define RAD2DEG(X) X / M_PI * 180.0

typedef struct 
{
    double x;
    double y;
    double heading;
} position_t;

void update_pos_odo_enc(position_t* pos, double Aleft_enc, double Aright_enc);
void update_pos_odo_acc(position_t* pos, position_t* speed, const double acc[3], double acc_mean[3], double Dleft_enc, double Dright_enc);
void update_heading_enc(double* heading, double Dleft_enc, double Dright_enc);
void init_odometry(int time_step);

#endif