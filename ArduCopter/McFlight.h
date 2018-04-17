#ifndef MCFLIGHT_H_
#define MCFLIGHT_H_

#include <ctime>
#include <stdint.h>

class McFlight {
private:
  enum {
    MF_INIT,
    MF_NOP,
    MF_MODE_STABILIZE,
    MF_MODE_GUIDED,
    MF_MODE_LOITER,
    MF_MODE_POSHOLD,
    MF_ARM,
    MF_TAKEOFF,
    MF_HOLD,
    MF_LAND,
    MF_FLY,
  };

  int state = MF_INIT;        // State initial.
  double wait = 0.0;          // Time to sleep in seconds.
  time_t timer;

  uint16_t roll_pwm = 0;
  uint16_t pitch_pwm = 0;
  uint16_t throttle_pwm = 0;
  uint16_t yaw_pwm = 0;
  
  void mf_sleep(double time_);
  void hold();
  void go_left(float distance, float time);
  void go_right(float distance, float time);
  void go_up(float distance, float time);
  void go_down(float distance, float time);
  void go_forward(float distance, float time);
  void go_backward(float distance, float time);

public:
  void run();
  void update_rc();
};

#endif
