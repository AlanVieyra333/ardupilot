#!/bin/bash
#
# chmod +x bash-commands.sh
# runtime: ./bash-commands.sh <ip-remote-host>

FILE_SH='arducopter'
FILE_LOG=logs.log

cmd_host=$1
cmd1="rm /data/ftp/internal_000/ardupilot/$FILE_SH"
cmd2="mv /data/ftp/$FILE_SH  /data/ftp/internal_000/ardupilot/$FILE_SH"
cmd3="chmod 755 /data/ftp/internal_000/ardupilot/$FILE_SH"
cmd4='sync'
cmd5='reboot'

(
  sleep 2
  echo ${cmd1}
  sleep 1
  echo ${cmd2}
  sleep 1
  echo ${cmd3}
  sleep 1
  echo ${cmd4}
  sleep 1
  echo ${cmd5}
  sleep 3
) | telnet ${cmd_host} 2>$FILE_LOG
