#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>

class Joystick {
private:
  bool joystick_enable;
  
  int16_t pwm_roll;               // Current pwm_roll. [1100, 1900]
  int16_t pwm_pitch;              // Current pwm_pitch. [1100, 1900]
  int16_t pwm_throttle;           // Current pwm_throttle. [1100, 1900]
  int16_t pwm_yaw;                // Current pwm_yaw. [1100, 1900]

  int16_t pwm_roll_final;         // Desired pwm_roll. [1100, 1900]
  int16_t pwm_pitch_final;        // Desired pwm_pitch. [1100, 1900]
  int16_t pwm_throttle_final;     // Desired pwm_throttle. [1100, 1900]
  int16_t pwm_yaw_final;          // Desired pwm_yaw. [1100, 1900]

  int16_t pwm_d = 1;              // Change differential. {1, 2, 4}

  void softChange();
  void softChangeRoll();
  void softChangePitch();
  void softChangeThrottle();
  void softChangeYaw();

public:
  void update();
  void setPWMRoll(int16_t _pwm_roll);
  void setPWMPitch(int16_t _pwm_pitch);
  void setPWMThrottle(int16_t _pwm_throttle);
  void setPWMYaw(int16_t _pwm_yaw);
  void setPWMD(int16_t _pwm_d);
  int16_t getPWMRoll();
  int16_t getPWMPitch();
  int16_t getPWMThrottle();
  int16_t getPWMYaw();
  int16_t getPWMD();
  void enable();
  void trim();
};

#endif
