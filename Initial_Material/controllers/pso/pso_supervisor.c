#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#include "../const.h"                     // TODO: put definitions in const.h file

#define ROBOTS 1
#define MAX_ROB 1
#define ROB_RAD 0.035
#define ARENA_SIZE 0.94               // TODO: change arena size

//#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
//#define SWARMSIZE 10                    // Number of particles in swarm (defined in pso.h)
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 40.0                       // Maximum velocity particle can attain
#define MININIT -20.0                   // Lower bound on initialization value
#define MAXINIT 20.0                    // Upper bound on initialization value
#define ITS 20                          // Number of iterations to run
//#define DATASIZE 2*(NB_SENSOR+2+1)      // Number of elements in particle (2 Neurons with 8 proximity sensors
                                        // + 2 recursive/lateral conenctions + 1 bias)
                                        // defined in pso.h

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_T 10                     // Time of simualtion to run for fitness during optimization

#define FINALRUNS 5
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8

static WbNodeRef robs[MAX_ROB];
WbDeviceTag rob_emitter[MAX_ROB];
WbDeviceTag met_emitter;
WbDeviceTag rec;
const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double new_loc[MAX_ROB][3];
double new_rot[MAX_ROB][4];


void calc_fitness(double[DATASIZE],double,int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);

/* RESET - Get device handles and starting locations */
void reset(void) {
  // Device variables
  char rob[] = "epuck0";
  char em_rob[] = "emitter0";
  char em_met[] = "emitter";
  char receive[] = "receiver";
  // char rob2[] = "rob10";
  // char em2[] = "emitter10";
  // char receive2[] = "receiver10";
  int i; //counter
  //For robot numbers < 10
  for (i=0;i<10 && i<MAX_ROB;i++) {
    robs[i] = wb_supervisor_node_get_from_def(rob);
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));       // TODO: put that also in metric supervisor
    new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
    rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
    new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
    rob_emitter[i] = wb_robot_get_device(em_rob);
    wb_emitter_set_channel(rob_emitter[i], 2);                        // communication channel from PSO supervisor to robots is 2
    if (rob_emitter[i]==0) printf("missing emitter %d\n",i);
    rob[5]++;
    em_rob[7]++;
  }
  rec = wb_robot_get_device(receive);
  wb_receiver_set_channel(rec, 1);
  met_emitter = wb_robot_get_device(em_met);
  wb_emitter_set_channel(met_emitter, 1);
  //For robot numbers < 20
  // for (i=10;i<20 && i<MAX_ROB;i++) {
  //   robs[i] = wb_supervisor_node_get_from_def(rob2);
  //   loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
  //   new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
  //   rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
  //   new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
  //   emitter[i] = wb_robot_get_device(em2);
  //   if (emitter[i]==0) printf("missing emitter %d\n",i);
  //   rec[i] = wb_robot_get_device(receive2);
  //   rob2[4]++;
  //   em2[8]++;
  //   receive2[9]++;
  // }
}

/* MAIN - Distribute and test controllers */
int main() {

  if (PSO) {
    double *weights;                         // Optimized result
    double buffer[255];                      // Buffer for emitter
    int i,j,k;                               // Counter variables

    /* Initialisation */
    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    reset();
    wb_receiver_enable(rec,32);       // TO VERIFY: baud rate ?

    wb_robot_step(256);


    double fit;                        // Fitness of the current FINALRUN
    double endfit;                     // Best fitness over 10 runs
    double w[DATASIZE];       // Weights to be send to robots (determined by pso() )
    double f;                 // Evaluated fitness (modified by calc_fitness() )
    double bestfit, bestw[DATASIZE];

    /* Evolve controllers */
    endfit = 0.0;
    bestfit = 0.0;

    // Do 10 runs and send the best controller found to the robot
    for (j=0;j<10;j++) {

      // Get result of optimization
      weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE);

      // Set robot weights to optimization results
      fit = 0.0;
      for (k=0;k<DATASIZE;k++)
          w[k] = weights[k];

      // Run FINALRUN tests and calculate average
      for (i=0;i<FINALRUNS;i++) {
        calc_fitness(w,f,FIT_T,MAX_ROB);
        fit += f;
      }
      fit /= FINALRUNS;

      // Check for new best fitness
      if (fit > bestfit) {
        bestfit = fit;
        for (i = 0; i < DATASIZE; i++){
          bestw[i] = weights[i];
        }
      }

      printf("Performance of the best solution: %.3f\n",fit);
      endfit += fit/10; // average over the 10 runs
    }
    printf("~~~~~~~~ Optimization finished.\n");
    printf("Best performance: %.3f\n",bestfit);
    printf("Average performance: %.3f\n",endfit);

    /* Send best controller to robots */
    for (j=0;j<DATASIZE;j++) {
      buffer[j] = bestw[j];
    }
    buffer[DATASIZE] = 1000000;
    for (i=0;i<ROBOTS;i++) {
      wb_emitter_send(rob_emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
    }
  }

  /* Wait forever */
  while (1) wb_robot_step(64);

  return 0;
}

// Makes sure no robots are overlapping        // TODO: add verification that there is no obstacle there
char valid_locs(int rob_id, int* j) {
  int i;
  if(rob_id > MAX_ROB){
    printf("Wrong robot ID\n");
    return 1;
  }
  for (i = 0; i < MAX_ROB; i++) {
    if (rob_id == i) continue;
    if (pow(new_loc[i][0]-new_loc[rob_id][0],2) +
	      pow(new_loc[i][2]-new_loc[rob_id][2],2) < (2*ROB_RAD+0.01)*(2*ROB_RAD+0.01)) {
      *j++; 
      return 0;
        }
  }
  return 1;
}

// Randomly position specified robot
void random_pos(int numRobs) {
  //printf("Setting random position for %d\n",rob_id);
  float leader_pos[2];
  leader_pos[0] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;
  leader_pos[1] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;

  int i;
  for (i=0;i<numRobs;i++) {
    new_rot[i][0] = 0.0;
    new_rot[i][1] = 1.0;
    new_rot[i][2] = 0.0;
    new_rot[i][3] = 2.0*3.14159*rnd();

    int j = 0;
    do {
      // disposition 2 by 2 0.5m apart
      new_loc[i][0] = leader_pos[0] + (i%2)*0.2 + j*0.03;    // add small values to find new places without robots and obstacles
      new_loc[i][2] = leader_pos[1] + (i/2)*0.2 + j*0.03;    // division will result in an integer 
      } 
      //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
    } while (!valid_locs(i,&j));

    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"), new_loc[i]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), new_rot[i]);
  }
}

// Distribute fitness functions among robots
void calc_fitness(double weights[DATASIZE], double fit, int fit_t, int numRobs) {
  double buffer[255];
  double *rbuffer;
  int i,j;

  /* Position all robots around one random spot*/
  random_pos(numRobs);

  /* Send data to robots */
  for (i=0;i<numRobs;i++) {
    for (j=0;j<DATASIZE;j++) {
      buffer[j] = weights[j];
    }
    buffer[DATASIZE] = fit_t; // set number of iterations at end of buffer
    wb_emitter_send(rob_emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));      // TODO: implement weights receiver at robot side
  }

  /* Send end criterion to metric supervisor */
  buffer[0] = fit_t;
  wb_emitter_send(met_emitter,(void *)buffer,(DATASIZE)*sizeof(double));

  /* Wait for response from supervisor */
  while (wb_receiver_get_queue_length(rec) == 0)
    wb_robot_step(64);

  /* Get fitness value */     
  rbuffer = (double *)wb_receiver_get_data(rec);
  fit = rbuffer[0];
  printf("PSO supervisor received fitness %f\n", fit);
}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[DATASIZE], double fit, int neighbors[SWARMSIZE][SWARMSIZE]) {

  calc_fitness(weights,fit,FIT_T,ROBOTS);

#if NEIGHBORHOOD == RAND_NB
  nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
  nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
  fixedRadius(neighbors,RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j) {
  return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

  int i,j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Clear old neighbors */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;

    /* Set new neighbors randomly */
    for (j = 0; j < numNB; j++)
      neighbors[i][(int)(SWARMSIZE*rnd())] = 1;

  }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

  int r[numNB];
  int tempRob;
  double dist[numNB];
  double tempDist;
  int i,j,k;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Clear neighbors */
    for (j = 0; j < numNB; j++)
      dist[j] = ARENA_SIZE;

    /* Find closest robots */
    for (j = 0; j < ROBOTS; j++) {

      /* Don't use self */
      if (i == j) continue;

      /* Check if smaller distance */
      if (dist[numNB-1] > robdist(i,j)) {
      	dist[numNB-1] = robdist(i,j);
      	r[numNB-1] = j;

      	/* Move new distance to proper place */
      	for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {

      	  tempDist = dist[k];
      	  dist[k] = dist[k-1];
      	  dist[k-1] = tempDist;
      	  tempRob = r[k];
      	  r[k] = r[k-1];
      	  r[k-1] = tempRob;

      	}
      }

    }

    /* Update neighbor table */
    for (j = 0; j < ROBOTS; j++)
      neighbors[i][j] = 0;
    for (j = 0; j < numNB; j++)
      neighbors[i][r[j]] = 1;

  }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {

  int i,j;

  /* Get neighbors for each robot */
  for (i = 0; i < ROBOTS; i++) {

    /* Find robots within range */
    for (j = 0; j < ROBOTS; j++) {

      if (i == j) continue;

      if (robdist(i,j) < radius) neighbors[i][j] = 1;
      else neighbors[i][j] = 0;

    }

  }

}

void step_rob() {
  wb_robot_step(64);
}
