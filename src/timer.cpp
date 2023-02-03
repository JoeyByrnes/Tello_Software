#include "timer.h"

extern struct timeval st, et;
extern struct timeval update_st, update_et;
extern FILE *log_file;
extern int update_delay;
extern int delay_microseconds;
extern int last_update_delay;

#define PERIOD_IN_MICROSECONDS 915

void startTimer(){
  gettimeofday(&st,NULL);
}

void stopTimer(){
  gettimeofday(&et,NULL);
  int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);

  //printf("%d\n",elapsed);
}

int stopTimer(FILE * file){
  gettimeofday(&et,NULL);
  int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);

  fprintf(file, "%d\n",elapsed);
  fflush(file);
  return elapsed;
}

int log_task_time(){
  int elapsed = stopTimer(log_file);
  //stopTimer();
  gettimeofday(&update_et,NULL);
  last_update_delay = ((update_et.tv_sec - update_st.tv_sec) * 1000000) + (update_et.tv_usec - update_st.tv_usec);
  gettimeofday(&update_st,NULL);
  startTimer();
  return elapsed;
}

void handle_periodic_task_scheduling(struct timespec &next){
  next.tv_nsec += PERIOD_IN_MICROSECONDS * 1000;
  if (next.tv_nsec >= 1000000000)
  {
      next.tv_nsec -= 1000000000;
      next.tv_sec += 1;
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
}


void handle_start_of_periodic_task(struct timespec &next){

  clock_gettime(CLOCK_MONOTONIC, &next);
}
void handle_end_of_periodic_task(struct timespec &next){

  next.tv_nsec += PERIOD_IN_MICROSECONDS * 1000;
  if (next.tv_nsec >= 1000000000)
  {
      next.tv_nsec -= 1000000000;
      next.tv_sec += 1;
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

}