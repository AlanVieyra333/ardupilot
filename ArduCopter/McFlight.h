#ifndef MCFLIGHT_H_
#define MCFLIGHT_H_

enum {
  MF_NOP,
  MF_MODE_STABILIZE,
  MF_MODE_GUIDED,
  MF_MODE_LOITER,
  MF_MODE_POSHOLD,
  MF_ARM,
  MF_TAKEOFF,
  MF_LAND,
  MF_FLY,
};

void mf_sleep(double time_);

#endif
