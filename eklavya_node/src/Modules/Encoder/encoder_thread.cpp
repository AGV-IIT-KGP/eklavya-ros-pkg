#include <stdio.h>
#include "../../eklavya2.h"
#include "encoder.h"

void *encoder_thread(void *arg) {
  while(1) {
    /* Fetch data from Shaft Encoder and load it in local vars */
//    printf ("ENCODER\n");
    
    pthread_mutex_lock(&pose_mutex);
    
    /* Update the pose_data using the data in local vars */
    
    pthread_mutex_unlock(&pose_mutex);
    
    usleep(10);
  }
}

