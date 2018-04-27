#include "Copter.h"

void Copter::mcflight_run() {
  mcflight.run();
}

void Copter::automatic_joystick_update() {
  joystick.update();
}
