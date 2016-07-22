#pragma once

#include <pthread.h>

#include "Thread.h"
#include "Functor.h"
#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10
#define LINUX_SCHEDULER_MAX_TIMESLICED_PROCS 10
#define LINUX_SCHEDULER_MAX_IO_PROCS 10
class RCInput;
class Scheduler{
 public:
  Scheduler();


  void     init();
  void     delay(uint16_t ms);
  void     delay_microseconds(uint16_t us);
  //  void     register_delay_callback(AP_HAL::Proc,
  //				   uint16_t min_time_ms);

  void     system_initialized();

  void     reboot(bool hold_in_bootloader);

  void     stop_clock(uint64_t time_usec);

  uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

  void microsleep(uint32_t usec);

 private:
  class SchedulerThread : public PeriodicThread {
 public:
   SchedulerThread(Thread::task_t t, Scheduler &sched)
     : PeriodicThread(t)
     , _sched(sched)
   { }

 protected:
   bool _run() override;

   Scheduler &_sched;
 };

 void _wait_all_threads();

 void     _debug_stack();

 // AP_HAL::Proc _delay_cb;
 // uint16_t _min_delay_cb_ms;

 // AP_HAL::Proc _failsafe;

 bool _initialized;
 pthread_barrier_t _initialized_barrier;

 uint8_t _max_freq_div;
 RCInput* rcin;
 public:
 void setRCInput(RCInput *_rcin);
 SchedulerThread _rcin_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_rcin_task, void), *this};

 void _rcin_task();

 uint64_t _stopped_clock_usec;
 uint64_t _last_stack_debug_msec;

};
