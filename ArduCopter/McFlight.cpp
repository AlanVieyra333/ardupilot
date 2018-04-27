/**
 * Task: McFlight - Autonomous flight
 * Author: Alan Fernando Rincón Vieyra
 * Version: 1.1 - TODO (Takeoff and land in mode STABILIZE)
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
  //copter.gcs_chan[0].send_message(MSG_BATTERY_STATUS);  // usar MSG_EXTENDED_STATUS1
  //copter.gcs_chan[0].send_message(MSG_ATTITUDE);  // Esto es su giroscopio
  //copter.gcs_chan[0].send_message(MSG_LOCATION);  // GLOBAL_POSITION_INT : Send long, lat, alt, speed x y z
  //copter.gcs_chan[0].send_message(MSG_LOCAL_POSITION);  // LOCAL_POSITION_NED // No usar porque es con respecto al origen
  //MSG_AHRS

  if (wait == 0.0 || difftime(time(0), timer) >= wait) {
    wait = 0.0; // Restart.

    // State machine.
    switch (state) {
      case MF_INIT:
        copter.joystick.enable();

        state = MF_MODE_STABILIZE;
        //state = MF_END;
        break;
      case MF_MODE_STABILIZE:
        if (copter.set_mode(STABILIZE, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode STABILIZE.");

          copter.joystick.setPWMRoll(copter.channel_roll->get_radio_trim());
          copter.joystick.setPWMPitch(copter.channel_pitch->get_radio_trim());
          copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_min());
          copter.joystick.setPWMYaw(copter.channel_yaw->get_radio_trim());
          copter.joystick.setPWMD(100);

          // Continuing with next state.
          state = MF_ARM;

          mf_sleep(1.0);
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 1.0);
        }

        break;
      case MF_ARM:
        // If disarmed, arm motors.
        if (copter.position_ok() && !copter.motors->armed()) {
          if (copter.init_arm_motors(true)) {
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Armed.");

            // Continuing with next state.
            state = MF_TAKEOFF_PHASE1;

            mf_sleep(2.0);
            copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 2.0);
          }
        }

        break;
      case MF_TAKEOFF_PHASE1:
        if (copter.motors->armed()) {
          takeoff_phase1();
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Takeoff phase1.");

          // Continuing with next state.
          state = MF_TAKEOFF_PHASE2;

          mf_sleep(1.0);
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 1.0);
        }

        break;
      case MF_TAKEOFF_PHASE2:
        if (copter.motors->armed()) {
          takeoff_phase2();
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Takeoff phase2.");

          // Continuing with next state.
          state = MF_MODE_POSHOLD;

          mf_sleep(2.0);
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 2.0);
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
          state = MF_HOLD;
        }

        break;
      case MF_HOLD:
        hold();
        copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Hold.");

        // Continuing with next state.
        state = MF_LAND;

        mf_sleep(5.0);
        copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 5.0);
        break;
      case MF_LAND:
        if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
          copter.gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Landing...");

          // Continuing with next state.
          state = MF_END;
        }

        break;
    }
  }
}

/**
 * Method to takeoff the drone (phase 1).
 * 
 * @returns void
*/
void McFlight::takeoff_phase1() {
  copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_trim() - 20);  // 380
  copter.joystick.setPWMD(4);
}

/**
 * Method to takeoff the drone (phase 2).
 * 
 * @param distance Distance in meters that the drone must move.
 * @returns void
*/
void McFlight::takeoff_phase2() {
  copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_trim() + 25);  // 650
  copter.joystick.setPWMD(1);
}

/**
 * Method to don't move the drone to any direction. 
 * 
 * @returns void
*/
void McFlight::hold() {
  copter.joystick.trim();
}

/**
 * Method to move the drone to left.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_left(float distance, float time) {}

/**
 * Method to move the drone to right.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_right(float distance, float time) {}

/**
 * Method to move the drone to up. In mode POSHOLD, the speed is 2.5 m/s.
 * above 60% of throttle stick.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_up(float distance, float time) {
  switch (copter.control_mode) {
    case LOITER:
    case POSHOLD:
      copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_max());
      copter.joystick.setPWMD(4);

      break;
    default:
      break;
  }
}

/**
 * Method to move the drone to down.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_down(float distance, float time) {}

/**
 * Method to move the drone to forward.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_forward(float distance, float time) {}

/**
 * Method to move the drone to backward.
 * 
 * @param distance Distance in meters that the drone must move.
 * @param time Time in seconds that the drone must take time to move.
 * @returns void
*/
void McFlight::go_backward(float distance, float time) {}

/**
 * Method to sleep the state machine of McFlight.
 * 
 * @param _time Time in seconds that will last asleep.
 * @returns void
*/
void McFlight::mf_sleep(double _time) {
  timer = time(0);
  wait = _time;
}
