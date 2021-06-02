#define ROBOTS_N 1

#define TASK 0 // Tasks: 0 is localization, 1 is flocking, 2 is formation control
#define TRAJECTORY 1 // Tractectory used if task is localization


/* ROBOT CONSTANTS */
#define NB_SENSORS 8
#define AXLE_LENGTH 0.052   // Distance between wheels of robot (meters)
#define WHEEL_AXIS 0.057    // Distance between the two wheels in meter
#define WHEEL_RADIUS 0.0205 // Radius of the wheel in meter
#define MAX_SPEED_WEB 6.28  // Maximum speed webots

#define MAX_SPEED 800
#define BIAS_SPEED 400
#define EPS 0.0001 // Epsilon


/* VERBOSE_FLAGS */
#define VERBOSE_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ACC false    	// Print odometry values computed with accelerometer
#define VERBOSE_GPS false    	// Print odometry values computed with accelerometer
#define VERBOSE_KALMAN false

#define SUPERVISOR_VERBOSE_METRIC false // print error on each timestep
#define SUPERVISOR_VERBOSE_POSITION false // print position on each timestep


/* FLOCKING */
#define FLOCK_SIZE 5
#define MIGR_X 2        // -0.1
#define MIGR_Y 0.5      //1.34     // SHOULD BE IN OUR REFERENTIAL: THE Y AXIS IS WEBOTS REVERSED Z AXIS

#define D_FLOCK 0.15    // TO DEFINE: targeted flocking distance
#define D_MAX_FLOCK 0.1 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed

#define COHESION_WEIGHT 0.15 //0.6
#define MIGRATION_WEIGHT 0.15
#define DISPERSION_WEIGHT 0.4
#define DISPERSION_THRESHOLD 0.2
#define SPEED_MOMENTUM 0.9


/* FORMATION */
#define LEADER_ID 0
#define D_MAX_FORM 2 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed


/* MATH HELPER */
#define sign(X) (2*(X >= 0) - 1)
