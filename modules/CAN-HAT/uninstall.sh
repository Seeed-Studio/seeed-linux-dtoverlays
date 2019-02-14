#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)" 1>&2
   exit 1
fi

is_Raspberry=$(cat /proc/device-tree/model | awk  '{print $1}')
if [ "x${is_Raspberry}" != "xRaspberry" ] ; then
  echo "Sorry, this drivers only works on raspberry pi"
  exit 1
fi

uname_r=$(uname -r)


echo "remove dtbos"
rm  /boot/overlays/2xMCP2517FD.dtbo  || true




echo "remove dkms"
rm  -rf /var/lib/dkms/mcp2517fd || true

echo "remove kernel modules"
rm  /lib/modules/${uname_r}/kernel/drivers/net/can/spi/mcp25xxfd-can.ko || true

echo "------------------------------------------------------"
echo "Please reboot your raspberry pi to apply all settings"
echo "Thank you!"
echo "------------------------------------------------------"
