/**
 * Task: McFlight - Autonomous flight
 * Author: Alan Fernando Rincón Vieyra
 * Version: 0.1 - TODO (Takeoff and land)
 * Date: Feb-2018
 * StateMachine: (MF_MODE_GUIDED) -> (MF_ARM) -> (MF_TAKEOFF) -> (MF_MODE_POSHOLD) -> (MF_LAND)
 * Compiling: git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
 * Simulation: ./ArduCopter/sim_vehicle.py -j4 --console
 * Note: View GCS_Mavlink.cpp & Copter.h
 */
#include "Copter.h"
#include "McFlight.h"
#include <ctime>

using namespace std;

int state = MF_MODE_POSHOLD;   // State initial. (q0)
double wait = 0.0;            // Time to sleep in seconds.
time_t timer;

void Copter::mcflight_run() {
  if (wait == 0.0 || difftime(time(0), timer) == wait) {
    wait = 0.0; // Restart.

    // State machine.
    switch (state) {
    case MF_ARM:
      // If disarmed, arm motors.
      if (copter.position_ok() && !copter.motors->armed()) {
        if (copter.init_arm_motors(true)) {
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Arming...");

          // Continuing with next state.
          state = MF_TAKEOFF;
        }
      }

      break;
    case MF_MODE_STABILIZE:
      if (copter.set_mode(STABILIZE, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode STABILIZE.");

        // Continuing with next state.
        state = MF_ARM;
      }

      break;
    case MF_MODE_GUIDED:
      if (copter.set_mode(GUIDED, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode GUIDED.");

        // Continuing with next state.
        state = MF_ARM;
      }

      break;
    case MF_MODE_LOITER:
      if (copter.set_mode(LOITER, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode LOITER.");

        // Continuing with next state.
        state = MF_ARM;
      }

      break;
    case MF_MODE_POSHOLD:
      if (copter.set_mode(POSHOLD, MODE_REASON_GCS_COMMAND)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Mode POSHOLD.");

        // Continuing with next state.
        state = MF_ARM;
      }

      break;
    case MF_TAKEOFF:
      if (copter.motors->armed()) {
        if(copter.do_user_takeoff(1*100, true)) {
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Takeoff.");

          // Continuing with next state.
          state = MF_LAND;

          mf_sleep(15.0);
          gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Waiting %f seconds...", 15.0);
        }
      }

      break;
    case MF_LAND:
      if (copter.set_mode(LAND, MODE_REASON_GCS_COMMAND)){
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "McFlight: Landing...");

        // Continuing with next state.
        state = MF_NOP;
      }

      break;
    }
  }
}

/**
 * Method for sleep the state machine of McFlight.
 * 
 * @param time_ Time in seconds that will last asleep.
 * @returns void
*/
void mf_sleep(double time_) {
  timer = time(0);
  wait = time_;
}
