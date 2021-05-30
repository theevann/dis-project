#define DATASIZE 6 
#define SWARMSIZE 10

// Functions
double *pso(int, int, double, double, double, double, double, int, int);                         // Run particle swarm optimization
void fitness(double[][DATASIZE], double[], int[][SWARMSIZE]);                                    // Fitness function for particle evolution
void findPerformance(double[][DATASIZE], double[], double[], char, int[][SWARMSIZE]);            // Find the current performance of the swarm
void updateLocalPerf(double[][DATASIZE], double[], double[][DATASIZE], double[], double[]);      // Update the best performance of a single particle
void updateNBPerf(double[][DATASIZE], double[], double[][DATASIZE], double[], int[][SWARMSIZE]); // Update the best performance of a particle neighborhood
void copyParticle(double[], double[]);                                                           // Copy value of one particle to another
double bestInSwarm(double[][DATASIZE], double[], double[]);                                      // Find the best result in a swarm
int argmax(double[], int);                                                                       // Find index of maximal value
int mod(int, int);                                                                               // Modulus function
double randIn(double, double);                                                                   // Generate random number in [vmin, vmax]
double s(double);                                                                                // S-function to transform [-infinity,infinity] to [0,1]
