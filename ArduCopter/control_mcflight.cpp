/*
 * TODO Task McFlight
git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
 */
#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

#define system_id 252
#define component_id 1
#define confirmation_id 101

enum{
  mcflight_start,
  mcflight_mode_guided,
  mcflight_takeoff,
  mcflight_run,
};
int mcflight_step = 0;

void Copter::mcflight_run() {
  mavlink_command_long_t packet;
  mavlink_message_t msg;

  memset(&packet, 0, sizeof(packet));
  packet.target_system = system_id;
  packet.target_component = component_id;
  packet.confirmation = confirmation_id;
  //**
  switch (mcflight_step) {
    case mcflight_start:
      if (copter.position_ok() && !copter.motors->armed()){
        gcs_chan[MAVLINK_COMM_0].send_text(MAV_SEVERITY_INFO, "McFlight enabled");

        packet.command = MAV_CMD_COMPONENT_ARM_DISARM;
        packet.param1 = 1.0f;

        mavlink_msg_command_long_encode(system_id, component_id, &msg, &packet);
        gcs_chan[MAVLINK_COMM_0].handleMessage(&msg);

        mcflight_step = -1;
      }
      break;
    /*case mcflight_mode_guided:
        packet.command = MAV_CMD_NAV_LOITER_UNLIM;
        packet.target_system = 223;
        packet.target_component = 34;
        packet.confirmation = 101;

        memcpy(_MAV_PAYLOAD_NON_CONST(&msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);

        msg.msgid = MAVLINK_MSG_ID_COMMAND_LONG;

        MAVAction(channel, &msg);

        mcflight_step = mcflight_run;
        break;
    case mcflight_takeoff:
      packet.param1 = 0.0;
      packet.param3 = 0.0;
      packet.param2 = 0.0;
      packet.param4 = 0.0;
      packet.param5 = 0.0;
      packet.param6 = 0.0;
      packet.param7 = 5.0;
      packet.command = MAV_CMD_NAV_TAKEOFF;
      packet.target_system = 223;
      packet.target_component = 34;
      packet.confirmation = 101;

      memcpy(_MAV_PAYLOAD_NON_CONST(&msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);

      msg.msgid = MAVLINK_MSG_ID_COMMAND_LONG;

      MAVAction(channel, &msg);
      break;
    case 11:
      msg.sysid = copter.g.sysid_my_gcs;

      msg.msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
      break;*/
  }
  //**/
}
