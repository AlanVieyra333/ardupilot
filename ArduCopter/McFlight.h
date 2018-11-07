#ifndef MCFLIGHT_H_
#define MCFLIGHT_H_

#include <stdint.h>

class McFlight {
private:
  enum {                            // List of states.
    MF_INIT,
    MF_NOP,
    MF_MODE_STABILIZE,
    MF_MODE_GUIDED,
    MF_MODE_LOITER,
    MF_MODE_POSHOLD,
    MF_ARM,
    MF_TAKEOFF_PHASE1,
    MF_TAKEOFF_PHASE2,
    MF_HOLD,
    MF_LAND,
    MF_FLY,
    MF_END,
  };

  int16_t state = MF_INIT;          // State initial.

  uint32_t wait = 0;                // Time to sleep in miliseconds.
  uint32_t timer;
  Location destiny;

  void takeoff_phase1();
  void takeoff_phase2();
  void hold();
  void go_left(float distance, float time);
  void go_right(float distance, float time);
  void go_up(float distance, float time);
  void go_down(float distance, float time);
  void go_forward(float distance, float time);
  void go_backward(float distance, float time);
  void mf_sleep(uint32_t time_);
  uint32_t mf_difftime(uint32_t a, uint32_t b);
  void mf_log(const char *fmt, ...);

public:
  void run();
  void set_destiny(Location);
};

#endif
