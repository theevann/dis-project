#ifndef FLOCKING_H
#define FLOCKING_H 

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "odometry.h"


void reynolds_rules();
void compute_wheels_speed(int *msl, int *msr);
void send_ping(void);
void process_received_ping_messages(void);
void process_flocking(position_t previous_position, position_t current_position, WbDeviceTag receiver, WbDeviceTag emitter, WbDeviceTag *des);
void limit(int *number, int limit);

#endif