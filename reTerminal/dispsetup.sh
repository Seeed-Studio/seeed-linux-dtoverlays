#!/bin/sh

CONNECTED=$(echo "`xrandr -q`" | grep -w 'connected' | awk '{print$1}')
CMDLINE="xrandr"

for i in $CONNECTED
do
  if [ $i = "DSI-1" ]; then
    CMDLINE="$CMDLINE --output DSI-1 --primary --mode 720x1280 --rate 60.00 --pos 0x0 --rotate right"
  elif [ $i = "HDMI-1" ]; then
	CMDLINE="$CMDLINE --output HDMI-1 --auto --pos 1280x0"
  fi
done

echo run:$CMDLINE
$CMDLINE