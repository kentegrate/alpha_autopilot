#include <alpha_drivers/decoder/Scheduler.h>

#include <algorithm>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>


#include <alpha_drivers/decoder/RCInput.h>
#include <alpha_drivers/decoder/common.h>
//#include "Storage.h"
//#include "Util.h"




#define APM_LINUX_RCIN_PRIORITY         13
#define APM_LINUX_MAIN_PRIORITY         12
#define APM_LINUX_IO_PRIORITY           10


#define APM_LINUX_RCIN_RATE             2000
#define APM_LINUX_TONEALARM_RATE        100
#define APM_LINUX_IO_RATE               50

#define SCHED_THREAD(name_, UPPER_NAME_)                        \
  {                                                           \
  .name = "sched-" #name_,                                \
    .thread = &_##name_##_thread,                           \
    .policy = SCHED_FIFO,                                   \
    .prio = APM_LINUX_##UPPER_NAME_##_PRIORITY,             \
    .rate = APM_LINUX_##UPPER_NAME_##_RATE,                 \
      }

Scheduler::Scheduler()
{ }
double get_mtime(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); 
}
double get_utime(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //micro秒を計算
  return ((double)(tv.tv_sec)*1000000 + (double)(tv.tv_usec)); 
}

void Scheduler::init()
{
  const struct sched_table {
    const char *name;
    SchedulerThread *thread;
    int policy;
    int prio;
    uint32_t rate;
  } sched_table[] = {
    SCHED_THREAD(rcin, RCIN),
  };

  mlockall(MCL_CURRENT|MCL_FUTURE);

  if (geteuid() != 0) {
    printf("WARNING: running as non-root. Will not use realtime scheduling\n");
  }

  struct sched_param param = { .sched_priority = APM_LINUX_MAIN_PRIORITY };
  sched_setscheduler(0, SCHED_FIFO, &param);

  /* set barrier to N + 1 threads: worker threads + main */
  unsigned n_threads = ARRAY_SIZE(sched_table) + 1;
  pthread_barrier_init(&_initialized_barrier, nullptr, n_threads);

  for (size_t i = 0; i < ARRAY_SIZE(sched_table); i++) {
    const struct sched_table *t = &sched_table[i];

    t->thread->set_rate(t->rate);
    t->thread->set_stack_size(256 * 1024);
    t->thread->start(t->name, t->policy, t->prio);
  }

#if defined(DEBUG_STACK) && DEBUG_STACK
  register_timer_process(FUNCTOR_BIND_MEMBER(&Scheduler::_debug_stack, void));
#endif
}

 void Scheduler::_debug_stack()
 {
  double now = get_mtime();

  if (now - _last_stack_debug_msec > 5000) {
  fprintf(stderr, "Stack Usage:\n"
	  "\trcin  = %zu\n",
	  _rcin_thread.get_stack_usage());
  _last_stack_debug_msec = now;
  }
 }

 void Scheduler::microsleep(uint32_t usec)
 {
  struct timespec ts;
  ts.tv_sec = 0;
  ts.tv_nsec = usec*1000UL;
  while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

 void Scheduler::delay(uint16_t ms)
 {
  if (_stopped_clock_usec) {
  return;
  }
  double start = get_mtime();

  while ((get_mtime() - start) < ms) {
  // this yields the CPU to other apps
    microsleep(1000);
    /*    if (_min_delay_cb_ms <= ms) {
      if (_delay_cb) {
	_delay_cb();
      }
      }*/
  }
}

 void Scheduler::delay_microseconds(uint16_t us)
 {
  if (_stopped_clock_usec) {
  return;
}
  microsleep(us);
}

/* void Scheduler::register_delay_callback(AP_HAL::Proc proc,
    uint16_t min_time_ms)
 {
  _delay_cb = proc;
  _min_delay_cb_ms = min_time_ms;
  }*/



void Scheduler::_rcin_task()
{

  rcin->_timer_tick();

}
void Scheduler::setRCInput(RCInput* _rcin)
{
  rcin = _rcin;
}



void Scheduler::_wait_all_threads()
{
  int r = pthread_barrier_wait(&_initialized_barrier);
  if (r == PTHREAD_BARRIER_SERIAL_THREAD) {
    pthread_barrier_destroy(&_initialized_barrier);
  }
}

void Scheduler::system_initialized()
{
  if (_initialized) {

  }

  _initialized = true;

  _wait_all_threads();
}


void Scheduler::stop_clock(uint64_t time_usec)
{
  if (time_usec >= _stopped_clock_usec) {
    _stopped_clock_usec = time_usec;
  }
}

bool Scheduler::SchedulerThread::_run()
{

  _sched._wait_all_threads();

    double next_run_usec = get_utime() + _period_usec;

    while (true) {
      double dt = next_run_usec - get_utime();
      if (dt > _period_usec) {
	// we've lost sync - restart
	next_run_usec = get_utime();
      } else {
	_sched.microsleep(dt);
      }
      next_run_usec += _period_usec;

      _task();
    }


    return true;
}
