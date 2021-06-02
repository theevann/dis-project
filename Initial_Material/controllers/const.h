#define ROBOTS_N 5
#define TRAJECTORY 1

#define MIGR_X 2        // -0.1
#define MIGR_Y 0.5      //1.34     // SHOULD BE IN OUR REFERENTIAL: THE Y AXIS IS WEBOTS REVERSED Z AXIS
#define D_FLOCK 0.15    // TO DEFINE: targeted flocking distance
#define D_MAX_FLOCK 0.1 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed

#define LEADER_ID 0
#define D_MAX_FORM 2 // TO DEFINE: maximum distance travelled in 1 timestep: timestep * maximum speed

#define SUPERVISOR_VERBOSE_METRIC false // print error on each timestep
