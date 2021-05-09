#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <webots/robot.h>

#include "odometry.h"


/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.0205		// Radius of the wheel in meter
#define ALPHA         	0.4	    	// Alpha running acc mean

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC true     	// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC true    	// Print odometry values computed with accelerometer

/*GLOBAL*/
static double T;

void update_pos_odo_acc(position_t* pos, position_t* speed, const double acc[3], double acc_mean[3], double Dleft_enc, double Dright_enc)
{
	double acc_wx = acc[1] - acc_mean[1];
	double acc_wy = -(acc[0] - acc_mean[0]);

	speed->x = speed->x + acc_wx * T;
	speed->y = speed->y + acc_wy * T;

	pos->x = pos->x + speed->x * T;
	pos->y = pos->y + speed->y * T;
	
	//  Compute heading with encoders
    Dleft_enc  = Dleft_enc * WHEEL_RADIUS;
	Dright_enc = Dright_enc * WHEEL_RADIUS;
	double omega = (Dright_enc - Dleft_enc) / WHEEL_AXIS / T;
	pos->heading = pos->heading + omega * T;

	// printf("%g\n", wb_robot_get_time());
    // printf("ACC WYWX %g, %g \n\n", acc_wy, acc_wx);
    // printf("ACC CURR %g, %g \n\n", acc[0], acc[1]);
    // printf("ACC MEAN %g, %g \n\n", acc_mean[0], acc_mean[1]);

	// KEEP TRACK OF ACCELEROMETER BIAS
	// if (acc_mean[0] == 0 && acc_mean[1] == 0) {
	// 	acc_mean[0] = acc[0];
	// 	acc_mean[1] = acc[1];
	// }
	// else {
	// 	if (abs(acc[0]) < 0.1 )
	// 		acc_mean[0] = ALPHA * acc[0] + (1-ALPHA) * acc_mean[0];
	// 	if (abs(acc[1]) < 0.1 )
	// 		acc_mean[1] = ALPHA * acc[1] + (1-ALPHA) * acc_mean[1];
	// }
	// TODO : Compute mean when not moving
	
	
	if (VERBOSE_ODO_ACC)
		printf("ODO with acceleration : %g %g %g\n", pos->x , pos->y , RAD2DEG(pos->heading));
}

void update_pos_odo_enc(position_t* pos, double Dleft_enc, double Dright_enc)
{
	//  Rad to meter : Convert the wheel encoders units into meters
    Dleft_enc  = Dleft_enc * WHEEL_RADIUS;
	Dright_enc = Dright_enc * WHEEL_RADIUS;

	// Comupute speeds : Compute the forward and the rotational speed
	double omega = (Dright_enc - Dleft_enc) / WHEEL_AXIS / T;
	double speed = (Dright_enc + Dleft_enc) / 2 / T;
	
	//  Compute the speed into the world frame (A) 
	double speed_wx = speed * cos(pos->heading);
	double speed_wy = speed * sin(pos->heading);

	// Integration : Euler method
	pos->x = pos->x + speed_wx * T;
	pos->y = pos->y + speed_wy * T;
	pos->heading = pos->heading + omega * T;

	if (VERBOSE_ODO_ENC)
    	printf("ODO with wheel encoders : %g %g %g\n", pos->x, pos->y, RAD2DEG(pos->heading));
}


void init_odometry(int time_step)
{
	T = time_step / 1000.0;
}