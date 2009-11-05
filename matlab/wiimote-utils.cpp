#include "wiimote.h"

double compute_relative_time (struct timespec timestamp, time_t capture_time)
{
	//fprintf(stderr,"Tiemstamp: sec: %ld nanosec: %ld comp: %f\n",timestamp.tv_sec - capture_time,timestamp.tv_nsec, comp);
	return (timestamp.tv_sec - capture_time + (double) ((double)timestamp.tv_nsec / (1e09) )) ;
}



