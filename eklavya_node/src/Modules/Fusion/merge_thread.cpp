#include <stdio.h>

#include "merge.h"

void *merge_thread(void *arg) {
  ros::Rate loop_rate(10);
  while(ros::ok()) {
      Merge m;
      m.laneLidar();
    loop_rate.sleep();
  }

  printf("Exiting merge code");
}
