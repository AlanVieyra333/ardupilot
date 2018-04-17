/**
 * Task: McFlight - Autonomous flight
 * Author: Alan Fernando Rincón Vieyra
 * Version: 0.2 - TODO (Takeoff and land in mode POSHOLD)
 * Date: Feb-2018
 * StateMachine: (MF_MODE_POSHOLD) -> (MF_ARM) -> (MF_TAKEOFF) -> (MF_LAND)
 * Compiling: git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
 * Simulation: ./ArduCopter/sim_vehicle.py -j4 --console -A "--uartA=uart:/dev/ttyUSB0"
 * Note: View GCS_Mavlink.cpp & Copter.h
 */
#include "McFlight.h"
#include "Copter.h"

void McFlight::run() {
  /*// Send value sensors.
  double alt = (copter.ahrs.get_home().alt + copter.current_loc.alt) * 10UL;
  alt /= 1000.0; // [m].
  const Vector3f &vel = copter.inertial_nav.get_velocity();
  Vector3f local_position;
  copter.ahrs.get_relative_position_NED_home(local_position);*/

  //*copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Bateria: %f", copter.battery.voltage());
  //*copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Altitud: %f", alt);
  //copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: GPS: %f %f %f %f", copter.current_loc.lat, copter.current_loc.lng, copter.ahrs.get_home().lat, copter.ahrs.get_home().lng);
  
  //copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: GPS: %f %f", local_position.x, local_position.y);
  //copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Velocidad: %f %f %f %f", vel.x, vel.y, vel.x, copter.ahrs.groundspeed());
  
  //copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Orientcion: %f", ??);
  copter.gcs_chan[0].send_message(MSG_EXTENDED_STATUS1); // usar MSG_EXTENDED_STATUS1
  //copter.gcs_chan[0].send_message(MSG_ATTITUDE);  // Esto es su giroscopio
  //copter.gcs_chan[0].send_message(MSG_LOCATION);  // GLOBAL_POSITION_INT : Send long, lat, alt, speed x y z
  //copter.gcs_chan[0].send_message(MSG_LOCAL_POSITION);  // LOCAL_POSITION_NED // No usar porque es con respecto al origen
  //MSG_AHRS

  if (wait == 0.0 || difftime(time(0), timer) >= wait) {
    wait = 0.0; // Restart.

    // State machine.
    switch (state) {
      case MF_INIT:
        //state = MF_MODE_POSHOLD;
        state = MF_NOP;
        break;
      case MF_ARM:
        // If disarmed, arm motors.
        if (copter.position_ok() && !copter.motors->armed()) {
          if (copter.init_arm_motors(true)) {
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Armed.");

            // Continuing with next state.
            state = MF_TAKEOFF;

            mf_sleep(2.0);
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 2.0);
          }
        }

        break;
      case MF_MODE_STABILIZE:
        if (copter.set_mode(STABILIZE, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode STABILIZE.");

          // Continuing with next state.
          state = MF_ARM;
        }

        break;
      case MF_MODE_GUIDED:
        if (copter.set_mode(GUIDED, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode GUIDED.");

          // Continuing with next state.
          state = MF_ARM;
        }

        break;
      case MF_MODE_LOITER:
        if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
          if (copter.control_mode == LOITER){
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode LOITER.");

            // Continuing with next state.
            state = MF_ARM;

            mf_sleep(1.0);
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 1.0);
          }
        }

        break;
      case MF_MODE_POSHOLD:
        if (copter.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode POSHOLD.");

          // Continuing with next state.
          state = MF_ARM;

          mf_sleep(1.0);
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 1.0);
        }

        break;
      case MF_TAKEOFF:
        if (copter.motors->armed()) {
          go_up(200.0, 3.0);

          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Takeoff.");

          // Continuing with next state.
          state = MF_HOLD;

          mf_sleep(8.5);
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 3.5);
        }

        break;
      case MF_HOLD:
        hold();

        // Continuing with next state.
        state = MF_LAND;

        mf_sleep(10.0);
        copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 10.0);

        break;
      case MF_LAND:
        if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Landing...");

          // Continuing with next state.
          state = MF_NOP;
        }

        break;
    }
  }
}

/**
 * Method to move the drone to down.
 * 
 * @returns void
*/
void McFlight::hold() {
  roll_pwm = 0;
  pitch_pwm = 0;
  throttle_pwm = 0;
  yaw_pwm = 0;
}

/**
 * Method to move the drone to left.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_left(float distance, float time_) {}

/**
 * Method to move the drone to right.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_right(float distance, float time_) {}

/**
 * Method to move the drone to up. In mode POSHOLD, max speed is the value of
 * g.pilot_velocity_z_max (default 2.5 m/s).
 * above 60% of throttle stick.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_up(float distance, float time_) {
  uint16_t pwm_max = copter.channel_throttle->get_radio_max();
  uint16_t pwm_trim = copter.channel_throttle->get_radio_trim();
  uint16_t pwm_vel = copter.g.pilot_velocity_z_max;
  float desired_velocity = distance / time_;

  throttle_pwm = (int)(desired_velocity * (pwm_max - pwm_trim) / (float)pwm_vel);

  if (throttle_pwm < (pwm_max - pwm_trim) * 0.2 ){
    throttle_pwm = (pwm_max - pwm_trim) * 0.2;
  } else if (throttle_pwm > (pwm_max - pwm_trim)) {
    throttle_pwm = pwm_max - pwm_trim;
  }

  //copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: speed: %d", throttle_pwm);
}

/**
 * Method to move the drone to down.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_down(float distance, float time_) {}

/**
 * Method to move the drone to forward.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_forward(float distance, float time_) {}

/**
 * Method to move the drone to backward.
 * 
 * @param distance Distance in centimeters that the drone must move.
 * @param time_ Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_backward(float distance, float time_) {}

/**
 * Method to sleep the state machine of McFlight.
 * 
 * @param time__ Time in seconds that will last asleep.
 * @returns void
*/
void McFlight::mf_sleep(double time_) {
  timer = time(0);
  wait = time_;
}

void McFlight::update_rc() {
  int16_t v[8];

  v[0] = copter.channel_roll->get_radio_trim() + roll_pwm;     // Roll.
  v[1] = copter.channel_pitch->get_radio_trim() + pitch_pwm;    // Pitch.
  v[2] = copter.channel_throttle->get_radio_trim() + throttle_pwm; // Throttle.
  v[3] = copter.channel_yaw->get_radio_trim() + yaw_pwm;      // Yaw.
  v[4] = 0;
  v[5] = 0;
  v[6] = 0;
  v[7] = 0;

  if (state != MF_NOP) {
    // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
    copter.failsafe.rc_override_active = hal.rcin->set_overrides(v, 8);

    // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
    copter.failsafe.last_heartbeat_ms = AP_HAL::millis(); //*/
  }
}
