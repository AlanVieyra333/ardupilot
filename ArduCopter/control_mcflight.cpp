/*
 * Task: McFlight - Autonomous flight
 * Author: Alan Fernando Rincón Vieyra
 * Version: 0.1 - TODO (Takeoff and land)
 * Date: Feb-2018
 * Compiling: git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
 * Simulation: ./ArduCopter/sim_vehicle.py -j4 --console
 * View GCS_Mavlink.cpp & Copter.h
 */
#include "Copter.h"
#include "McFlight.h"
#include <ctime>

using namespace std;

int state = MCFLIGHT_MODE_GUIDED;
time_t start;

void Copter::mcflight_run() {
  // State machine.
  switch (state) {
    case MCFLIGHT_ARM:
      if (copter.position_ok() && !copter.motors->armed()) {
          // if disarmed, arm motors
          copter.init_arm_motors(true);
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Arming...");

          // Continuing with next state.
          state = MCFLIGHT_TAKEOFF;
      }

      break;
    case MCFLIGHT_MODE_STABILIZE:
      if (copter.set_mode(STABILIZE, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode STABILIZE.");

        // Continuing with next state.
        state = MCFLIGHT_ARM;
      }
      break;
    case MCFLIGHT_MODE_GUIDED:
      if (copter.set_mode(GUIDED, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode GUIDED.");

        // Continuing with next state.
        state = MCFLIGHT_ARM;
      }

      break;
    case MCFLIGHT_MODE_LOITER:
      //if (difftime( time(0), start) == 6.0)
      if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode LOITER.");

        // Continuing with next state.
        state = MCFLIGHT_ARM;
      }

      break;
    case MCFLIGHT_MODE_POSHOLD:
      if (difftime( time(0), start) == 7.0)
        if (copter.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode POSHOLD.");

          // Continuing with next state.
          state = MCFLIGHT_LAND;
        }

      break;
    case MCFLIGHT_TAKEOFF:
      if (copter.motors->armed()) {
        if(copter.do_user_takeoff(1*100, true)) {
          start = time(0);

          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Takeoff. \nWaiting 15 seconds...");

          // Continuing with next state.
          state = MCFLIGHT_MODE_POSHOLD;
        }
      }

      break;
    case MCFLIGHT_LAND:
      if (difftime( time(0), start) == 17.0) {
        if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)){
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Landing...");

          // Continuing with next state.
          state = MCFLIGHT_NOP;
        }
      }

      break;
  }

  // (Delete) Show message indicating all ok.
  //gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Runing...");
}
