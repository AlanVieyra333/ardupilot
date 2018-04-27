#include "Joystick.h"
#include "Copter.h"

void Joystick::softChange() {
  softChangeRoll();
  softChangePitch();
  softChangeThrottle();
  softChangeYaw();
}

void Joystick::softChangeRoll() {
  if (pwm_roll != pwm_roll_final){
    if (pwm_roll_final > pwm_roll) {
      pwm_roll += pwm_d;

      if (pwm_roll_final < pwm_roll) {
        pwm_roll = pwm_roll_final;
      }
    } else {
      pwm_roll -= pwm_d;

      if (pwm_roll_final > pwm_roll) {
        pwm_roll = pwm_roll_final;
      }
    }
  }
}

void Joystick::softChangePitch() {
  if (pwm_pitch != pwm_pitch_final){
    if (pwm_pitch_final > pwm_pitch) {
      pwm_pitch += pwm_d;

      if (pwm_pitch_final < pwm_pitch) {
        pwm_pitch = pwm_pitch_final;
      }
    } else {
      pwm_pitch -= pwm_d;

      if (pwm_pitch_final > pwm_pitch) {
        pwm_pitch = pwm_pitch_final;
      }
    }
  }
}

void Joystick::softChangeThrottle() {
  if (pwm_throttle != pwm_throttle_final){
    if (pwm_throttle_final > pwm_throttle) {
      pwm_throttle += pwm_d;

      if (pwm_throttle_final < pwm_throttle) {
        pwm_throttle = pwm_throttle_final;
      }
    } else {
      pwm_throttle -= pwm_d;

      if (pwm_throttle_final > pwm_throttle) {
        pwm_throttle = pwm_throttle_final;
      }
    }
  }
}

void Joystick::softChangeYaw() {
  if (pwm_yaw != pwm_yaw_final){
    if (pwm_yaw_final > pwm_yaw) {
      pwm_yaw += pwm_d;

      if (pwm_yaw_final < pwm_yaw) {
        pwm_yaw = pwm_yaw_final;
      }
    } else {
      pwm_yaw -= pwm_d;

      if (pwm_yaw_final > pwm_yaw) {
        pwm_yaw = pwm_yaw_final;
      }
    }
  }
}

void Joystick::update() {
  if (joystick_enable) {
    int16_t v[8];

    softChange();

    v[0] = pwm_roll;          // Roll channel.
    v[1] = pwm_pitch;         // Pitch channel.
    v[2] = pwm_throttle;      // Throttle channel.
    v[3] = pwm_yaw;           // Yaw channel.
    v[4] = 0;
    v[5] = 0;
    v[6] = 0;
    v[7] = 0;

    // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
    copter.failsafe.rc_override_active = hal.rcin->set_overrides(v, 8);

    // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
    copter.failsafe.last_heartbeat_ms = AP_HAL::millis(); //*/
  }
}

void Joystick::setPWMRoll(int16_t _pwm_roll) {
  pwm_roll_final = _pwm_roll;
}

void Joystick::setPWMPitch(int16_t _pwm_pitch) {
  pwm_pitch_final = _pwm_pitch;
}

void Joystick::setPWMThrottle(int16_t _pwm_throttle) {
  pwm_throttle_final = _pwm_throttle;
}

void Joystick::setPWMYaw(int16_t _pwm_yaw) {
  pwm_yaw_final = _pwm_yaw;
}

void Joystick::setPWMD(int16_t _pwm_d) {
  pwm_d = _pwm_d;
}

void Joystick::enable() {
  joystick_enable = true;
}

void Joystick::trim() {
  pwm_roll_final = copter.channel_roll->get_radio_trim();
  pwm_pitch_final = copter.channel_pitch->get_radio_trim();
  pwm_throttle_final = copter.channel_throttle->get_radio_trim();
  pwm_yaw_final = copter.channel_yaw->get_radio_trim();
  pwm_d = 4;
}
