/*
 * File:          supervisor_local.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define TASK "localization"

WbNodeRef rob;		           // Robot node
WbFieldRef rob_trans;	                      // Robots translation fields
WbFieldRef rob_rotation;	           // Robots rotation fields
WbDeviceTag receiver;			// Single receiver

static int time_step;
static int t;
float loc_abs[3];		// Aboslute Location of the robot
float loc_est[3];               // Estimated position


/*
 * Initialize supervisor for getting robot absolute position
 */
void init_super(void) {
	wb_robot_init();
           
           time_step = wb_robot_get_basic_time_step();
           t = 0;
           
	receiver = wb_robot_get_device("receiver");
	if (receiver==0) printf("missing receiver in supervisor\n");
	
	wb_receiver_enable(receiver, time_step);
	
	char rob_name[7] = "ROBOT1"; // verify
	rob = wb_supervisor_node_get_from_def(rob_name);
	rob_trans = wb_supervisor_node_get_field(rob,"translation");
	rob_rotation = wb_supervisor_node_get_field(rob,"rotation");
}


/*
 * Compute performance metric.
 
void compute_fitness(float* fit_c, float* fit_o) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i; int j;
	for (i=0;i<FLOCK_SIZE;i++) {
		for (j=i+1;j<FLOCK_SIZE;j++) {	
			// Distance measure for each pair ob robots
			*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		}

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2]-orient_migr);
		*fit_o += angle_diff > M_PI ? 2*M_PI-angle_diff : angle_diff;
	}
	*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_o /= FLOCK_SIZE;
}
*/
void get_absolute_position(void) {
            
           // Get data
	loc_abs[0] = wb_supervisor_field_get_sf_vec3f(rob_trans)[0]; // X
	loc_abs[1] = wb_supervisor_field_get_sf_vec3f(rob_trans)[2]; // Z
	loc_abs[2] = wb_supervisor_field_get_sf_rotation(rob_rotation)[3]; // THETA 			
}

void get_info(void) {
          char *inbuffer;

          while (wb_receiver_get_queue_length(receiver) > 0) {
            inbuffer = (char*) wb_receiver_get_data(receiver);
            sscanf(inbuffer,"%f#%f#%f",&loc_est[0],&loc_est[1],&loc_est[2]);
	
            wb_receiver_next_packet(receiver);
    }

}

void compute_metric(void) { 
          printf("Estimated position is x: %f, y: %f, theta: %f\n",loc_est[0],loc_est[1],loc_est[2]);

}

/*
 * Main function.
 */
 
int main() {
           
	init_super();
	// printf("Channel Emitter %d\n", wb_emitter_get_channel(emitter));
	
	
	for(;;) {
		// Get absolute position of the robot
		get_absolute_position() ;
		
		// Receive estimated position of the robot
		get_info();
		
		// Compute metric
		compute_metric();
		
		wb_robot_step(time_step);
                      t += time_step;
	}
}
