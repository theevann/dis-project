#ifndef ODOMETRY_H
#define ODOMETRY_H 

#define RAD2DEG(X) X / M_PI * 180.0

typedef struct 
{
    double x;
    double y;
    double heading;
} position_t;

void calibrate_acc(double* acc, double* acc_mean);
void get_acc(WbDeviceTag device, double* acc);
void get_encoder(WbDeviceTag device_left, WbDeviceTag device_right, double* prev_left_enc, double* prev_right_enc, double* left_enc, double* right_enc);
void get_gps(WbDeviceTag device, double* last_gps_time_sec, double* prev_gps, double* gps, bool* gps_true);
void update_pos_enc(position_t* pos, double Aleft_enc, double Aright_enc);
void update_pos_acc(position_t* pos, position_t* speed, const double acc[3], double acc_mean[3], double Dleft_enc, double Dright_enc);
void update_pos_gps(position_t *pos, double* last_gps_time_sec, double* prev_gps, double* gps);
void update_heading_enc(double* heading, double Dleft_enc, double Dright_enc);
void restrict_heading(double* heading);
void init_odometry(int time_step);

#endif