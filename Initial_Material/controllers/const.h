#include <math.h>
#include "struct.h"

#define TASK 1   // TASK: 0 is localization, 1 is flocking, 2 is formation control
#define WORLD 1  // WORLD: 0 is localization, 1 is obstacles, 2 is crossing
#define TRAJECTORY 1 // Tractectory used if task is localization

// PSO 
#define PSO true
#define ARENA_SIZE_X 5 // ARENA IS A BIT BIGGER BUT THEN ROBOT MIGHT BE PLACED OUTSIDE !
#define ARENA_SIZE_Y 3 // ARENA IS A BIT BIGGER BUT THEN ROBOT MIGHT BE PLACED OUTSIDE !


/* PSO definitions */
#define PSO_N_RESTART 5       // Number of PSO restart
#define PSO_ITS 20         // Number of PSO iterations to run
#define PSO_FIT_T 20       // Time of simulation to run for fitness during optimization
#define SWARMSIZE 5 
#define NB 1          // Number of neighbors on each side
#define LWEIGHT 2.0   // Weight of attraction to personal best
#define NBWEIGHT 2.0  // Weight of attraction to neighborhood best
#define VMAX 1.0      // Maximum velocity particle can attain
#define MININIT 0.0   // Lower bound on initialization value
#define MAXINIT 1.0   // Upper bound on initialization value



#if TASK == 0  // if localization then...
    #define DATASIZE 0
#elif TASK == 1 // if flocking then...
    #define DATASIZE 5  
#elif TASK == 2 // if formation them...
    #define DATASIZE 0
#endif


// These variables are mostly useless for WORLD 0 but need to be defined
#if WORLD == 0
    #define N_ROBOTS 1
    #define FLOCK_SIZE 1
    #define TIME_IN_OBSTACLE_AVOIDANCE 0
    #define PSO_RAND_MIGRPOS 0
#elif WORLD == 1
    #define N_ROBOTS 5
    #define FLOCK_SIZE 5
    #define TIME_IN_OBSTACLE_AVOIDANCE 2
    #define PSO_RAND_MIGRPOS 1
#elif WORLD == 2
    #define N_ROBOTS 10
    #define FLOCK_SIZE 5
    #define TIME_IN_OBSTACLE_AVOIDANCE 1
    #define PSO_RAND_MIGRPOS 0
#endif


/* ROBOT CONSTANTS */
#define NB_SENSORS 8
#define AXLE_LENGTH 0.052   // Distance between wheels of robot (meters)
#define WHEEL_AXIS 0.057    // Distance between the two wheels in meter
#define WHEEL_RADIUS 0.0205 // Radius of the wheel in meter
#define MAX_SPEED_WEB 6.28  // Maximum speed webots
#define ROB_RADIUS 0.035    // Robot radius

#define MAX_SPEED 800
#define BIAS_SPEED 400
#define EPS 0.0001 // Epsilon


/* VERBOSE_FLAGS */
#define VERBOSE_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ACC false    	// Print odometry values computed with accelerometer
#define VERBOSE_GPS false    	// Print odometry values computed with accelerometer
#define VERBOSE_KALMAN false
#define VERBOSE_MESSAGING false

#define SUPERVISOR_VERBOSE_METRIC false // print error on each timestep
#define SUPERVISOR_VERBOSE_POSITION false // print position on each timestep


/* FLOCKING */
#define N_FLOCKING_GROUP (N_ROBOTS / FLOCK_SIZE)

extern const position_t MIGR[];
extern const position_t INIT_POS[];
extern const float FORM_REL_POS[5][2];

// KEEP TOGETHER WITH HIGHER INTERROBOT DISTANCE
#define COHESION_WEIGHT 0.2
#define MIGRATION_WEIGHT 0.2
#define DISPERSION_WEIGHT 0.4
#define DISPERSION_THRESHOLD 0.3
#define SPEED_MOMENTUM 0.9

// GOES FASTER TOWARD GOAL
// #define COHESION_WEIGHT 0.15
// #define MIGRATION_WEIGHT 0.2
// #define DISPERSION_WEIGHT 0.4
// #define DISPERSION_THRESHOLD 0.2 // 0.3 for W2
// #define SPEED_MOMENTUM 0.2

// #define COHESION_NORMALISATION 0  // also try with: (DISPERSION_THRESHOLD*2)
#define COHESION_NORMALISATION (DISPERSION_THRESHOLD*2)


#define D_FLOCK DISPERSION_THRESHOLD    // Targeted flocking distance // TODO:
#define D_MAX_FLOCK 0.13 // TODO: check - maximum distance travelled in 1 timestep: timestep * maximum speed


/* FORMATION */
#define LEADER_ID 0
#define D_MAX_FORM 0.1 // TODO: maximum distance travelled in 1 timestep: timestep * maximum speed


/* MATH HELPER */
#define sign(X) (2*(X >= 0) - 1)
