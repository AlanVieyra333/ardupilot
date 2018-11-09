/**
 * Task: McFlight - Autonomous flight
 * Author: Alan Fernando Rincón Vieyra
 * Version: 1.1 - TODO (Takeoff and land in mode STABILIZE)
 * Date: Feb-2018
 * StateMachine: (MF_MODE_POSHOLD) -> (MF_ARM) -> (MF_TAKEOFF) -> (MF_LAND)
 * Compiling: git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
 * Simulation: cd ./ArduCopter && sim_vehicle.py -j4 --console -A "--uartA=uart:/dev/ttyUSB0"
 * Note: View GCS_Mavlink.cpp & Copter.h
 */

#include "Copter.h"
#include "McFlight.h"

void Copter::mcflight_run() {
  mcflight.run();
}

void Copter::automatic_joystick_update() {
  joystick.update();
}

void McFlight::mf_log(const char *fmt, ...) {
  va_list arg_list;
  va_start(arg_list, fmt);
  gcs().send_textv(MAV_SEVERITY_INFO, fmt, arg_list);
  va_end(arg_list);
}

void McFlight::run() {
  /*/ Send value sensors.
  double alt = (copter.ahrs.get_home().alt + copter.current_loc.alt) * 10UL;
  alt /= 1000.0; // [m].
  const Vector3f &vel = copter.inertial_nav.get_velocity();
  //Vector3f local_position;
  //copter.ahrs.get_relative_position_NED_home(local_position);

  mf_log("McFlight: Bateria: %f", copter.battery.voltage());
  mf_log("McFlight: Destiny - latitude: %d", this->destiny.lat);
  mf_log("McFlight: Destiny - longitude: %d", this->destiny.lng);

  if (copter.g2.proximity.get_status() == AP_Proximity::Proximity_Good) {
    mf_log("McFlight: Proximity Good");

    float distance;
    copter.g2.proximity.get_horizontal_distance(0.0, distance);
    mf_log("McFlight: Distance-forward: %d", (uint16_t)(distance*100));

  } else {
    mf_log("McFlight: Proximity BAD");
  }

  mf_log("McFlight: GPS: %f %f  / %f %f", copter.current_loc.lat, copter.current_loc.lng, copter.ahrs.get_home().lat, copter.ahrs.get_home().lng);
  mf_log("McFlight: Altitud: %f", alt);
  //mf_log("McFlight: GPS: %f %f", local_position.x, local_position.y);
  mf_log("McFlight: Velocidad: %f %f %f %f", vel.x, vel.y, vel.x, copter.ahrs.groundspeed());
  
  //mf_log("McFlight: Orientcion: %f", ??);
  //copter.gcs_chan[0].send_message(MSG_BATTERY_STATUS);  // usar MSG_EXTENDED_STATUS1
  //copter.gcs_chan[0].send_message(MSG_ATTITUDE);  // Esto es su giroscopio
  //copter.gcs_chan[0].send_message(MSG_LOCATION);  // GLOBAL_POSITION_INT : Send long, lat, alt, speed x y z
  //copter.gcs_chan[0].send_message(MSG_LOCAL_POSITION);  // LOCAL_POSITION_NED // No usar porque es con respecto al origen
  //MSG_AHRS*/

  gcs().send_message(MSG_MCFLIGHT_DRONE_STATUS);

  if (this->wait == 0 || mf_difftime(AP_HAL::millis(), this->timer) >= this->wait) {
    this->wait = 0; // Restart.

    // State machine.
    switch (this->state) {
      case MF_INIT:
        copter.joystick.enable();

        state = MF_MODE_STABILIZE;
        //state = MF_END;
        break;
      case MF_MODE_STABILIZE:
        if (copter.set_mode(STABILIZE, MODE_REASON_GCS_COMMAND)) {
          mf_log("McFlight: Mode STABILIZE.");

          copter.joystick.setPWMRoll(copter.channel_roll->get_radio_trim());
          copter.joystick.setPWMPitch(copter.channel_pitch->get_radio_trim());
          copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_min());
          copter.joystick.setPWMYaw(copter.channel_yaw->get_radio_trim());
          copter.joystick.setPWMD(100);

          // Continuing with next state.
          state = MF_ARM;

          mf_sleep(1000);
          mf_log("McFlight: Waiting %f seconds...", 1.0);
        }

        break;
      case MF_ARM:
        // If disarmed, arm motors.
        if (copter.position_ok() && !copter.motors->armed()) {
          if (copter.init_arm_motors(true)) {
            mf_log("McFlight: Armed.");

            // Continuing with next state.
            state = MF_TAKEOFF_PHASE1;

            mf_sleep(2000);
            mf_log("McFlight: Waiting %f seconds...", 2.0);
          }
        }

        break;
      case MF_TAKEOFF_PHASE1:
        if (copter.motors->armed()) {
          takeoff_phase1();
          mf_log("McFlight: Takeoff phase1.");

          // Continuing with next state.
          state = MF_TAKEOFF_PHASE2;

          mf_sleep(1000);
          mf_log("McFlight: Waiting %f seconds...", 1.0);
        }

        break;
      case MF_TAKEOFF_PHASE2:
        if (copter.motors->armed()) {
          takeoff_phase2();
          mf_log("McFlight: Takeoff phase2.");

          // Continuing with next state.
          state = MF_MODE_POSHOLD;

          mf_sleep(2000);
          mf_log("McFlight: Waiting %f seconds...", 2.0);
        }

        break;
      case MF_MODE_POSHOLD:
        if (copter.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
          mf_log("McFlight: Mode POSHOLD.");

          // Continuing with next state.
          state = MF_HOLD;
        }

        break;
      case MF_HOLD:
        hold();
        mf_log("McFlight: Hold.");

        if (copter.joystick.getPWMPitch() == copter.channel_pitch->get_radio_trim()) {
          // Continuing with next state.
          state = MF_LAND;

          mf_sleep(10000);
          mf_log("McFlight: Waiting %f seconds...", 10.0);
        } else {
          // Continuing with next state.
          state = MF_LAND;

          mf_sleep(2000);
          mf_log("McFlight: Waiting %f seconds...", 2.0);
        }
        
        break;
      case MF_FLY:
        //if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
          mf_log("McFlight: FLYING...");

	        go_forward(0, 0);

          // Continuing with next state.
          state = MF_HOLD;

          mf_sleep(2000);
          mf_log("McFlight:  Waiting %f seconds...", 2.0);
	  
        //}

        break;
      case MF_LAND:
        if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)) {
          mf_log("McFlight: Landing...");

          // Continuing with next state.
          state = MF_END;
        }

        break;
    }
  }
}

/**
 * Method to set the destiny location.
 * 
 * @returns void
*/
void McFlight::set_destiny(Location loc) {
  this->destiny = loc;
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
  copter.joystick.setPWMThrottle(copter.channel_throttle->get_radio_trim() + 75);  // 561
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
void McFlight::go_forward(float distance, float time) {
	copter.joystick.setPWMPitch(copter.channel_pitch->get_radio_min());  //  
	copter.joystick.setPWMD(2);
}

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
void McFlight::mf_sleep(uint32_t wait) {
  this->timer = AP_HAL::millis();
  this->wait = wait;
}

uint32_t McFlight::mf_difftime(uint32_t a, uint32_t b) {
  if (a < b) {
    return 0;
  } else {
    return a - b;
  }
}