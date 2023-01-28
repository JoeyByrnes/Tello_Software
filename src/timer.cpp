#include "timer.h"

extern struct timeval st, et;
extern struct timeval update_st, update_et;
extern FILE *log_file;
extern int update_delay;
extern int delay_microseconds;
extern int last_update_delay;

void startTimer(){
  gettimeofday(&st,NULL);
}

void stopTimer(){
  gettimeofday(&et,NULL);
  int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);

  printf("%d\n",elapsed);
}

void stopTimer(FILE * file){
  gettimeofday(&et,NULL);
  int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);

  fprintf(file, "%d\n",elapsed);
  fflush(file);
}

void handle_start_of_periodic_task(){
  stopTimer(log_file);
  gettimeofday(&update_et,NULL);
  last_update_delay = ((update_et.tv_sec - update_st.tv_sec) * 1000000) + (update_et.tv_usec - update_st.tv_usec);
  gettimeofday(&update_st,NULL);
  startTimer();
}
void handle_end_of_periodic_task(){
  gettimeofday(&update_et,NULL);
  int elapsed_microseconds = ((update_et.tv_sec - update_st.tv_sec) * 1000000) + (update_et.tv_usec - update_st.tv_usec);
  update_delay = (update_delay + last_update_delay) / 2.0;
  if(update_delay > (MICROSEC_PER_SEC/UPDATE_HZ)){
    delay_microseconds--;
  }
  else if(update_delay < (MICROSEC_PER_SEC/UPDATE_HZ)-2){
    delay_microseconds++;
  }
  int us_to_delay = delay_microseconds;
  if(us_to_delay < 10){
    us_to_delay = 10; // minimum delay to avoid using 100% of processor
  }
  usleep(us_to_delay);
}