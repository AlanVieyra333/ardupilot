#!/bin/sh
HOST='192.168.42.1'
DIR='build/bebop/bin'
FILE='arducopter'

cd $DIR
ftp -n $HOST <<END_SCRIPT
put $FILE
quit
END_SCRIPT
exit 0
