#!/bin/bash


# Common path
MOD_PATH=`pwd`/modules
CFG_PATH=/boot/config.txt
INS_PATH=/lib/modules/`uname -r`/extra/seeed
KBUILD=/lib/modules/`uname -r`/build

# Check root
if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (use sudo)" 1>&2
  exit 1;
fi

# Check headers
function check_kernel_headers() {
  RPI_HDR=/usr/src/linux-headers-`uname -r`

  if [ -d $RPI_HDR ]; then
    echo "Installed: $RPI_HDR"
    return 0;
  fi

  echo " !!! Your kernel version is `uname -r`"
  echo "     Couldn't find *** corresponding *** kernel headers with apt-get."
  echo "     This may happen if you ran 'rpi-update'."
  echo " Choose  *** y *** to install kernel-headers to version `uname -r` and continue."
  echo " Choose  *** N *** to exit without this driver support, by default."
  read -p "Would you like to proceed? (y/N)" -n 1 -r -s
  echo
  if ! [[ $REPLY =~ ^[Yy]$ ]]; then
    exit 1;
  fi

  apt-get -y install raspberrypi-kernel-headers
}

# Build module
function build_modules {
  if [ $# -eq 0 ]; then
    echo "No module to compile!"
    exit 1;
  fi

  for i
  do
    RET=1
    make -C $KBUILD M=$MOD_PATH/$i || RET=0
    if [ $RET -eq 0 ]; then
      echo Build failed: $i
      exit 1
    fi
  done
}

function clean_modules {
  if [ $# -eq 0 ]; then
    echo "No module to clean!"
    exit 1;
  fi

  for i
  do
    make -C $KBUILD M=$MOD_PATH/$i clean || echo Clean failed: $i;
  done
}

# Install module
function install_modules {
  if [ $# -eq 0 ]; then
    echo "No module to install!"
    exit 1;
  fi

  mkdir -p $INS_PATH;

  for i
  do
    if [ $(echo $i | grep -c "/") -eq 0 ]; then
      cp -fv $MOD_PATH/$i/$i.ko $INS_PATH || exit 1;
    else
      cp -fv $MOD_PATH/$i.ko $INS_PATH || exit 1;
    fi
  done
}

function uninstall_modules { 
  if [ $# -eq 0 ]; then
    echo "No module to uninstall!"
    exit 1;
  fi

  for i
  do
    if [ -e $INS_PATH/$i.ko ]; then
      rm -fv $INS_PATH/$i.ko
    fi
  done
}

# Overlay
function install_overlay {
  if [ $# -eq 0 ]; then
    echo "No dtbo to install!"
    exit 1;
  fi

  grep -q "^ignore_lcd=1$" $CFG_PATH || \
    echo "ignore_lcd=1" >> $CFG_PATH
  grep -q "^enable_uart=1$" $CFG_PATH || \
    echo "enable_uart=1" >> $CFG_PATH
  grep -q "^dtoverlay=dwc2,dr_mode=host$" $CFG_PATH || \
    echo "dtoverlay=dwc2,dr_mode=host" >> $CFG_PATH
  grep -q "^dtoverlay=vc4-kms-v3d-pi4$" $CFG_PATH || \
    echo "dtoverlay=vc4-kms-v3d-pi4" >> $CFG_PATH

  for i
  do
    make overlays/rpi/$i-overlay.dtbo || exit 1;
    cp -fv overlays/rpi/$i-overlay.dtbo /boot/overlays/$i.dtbo || exit 1;
	
	grep -q "^dtoverlay=$i$" $CFG_PATH || \
	  echo "dtoverlay=$i" >> $CFG_PATH
  done
}

function uninstall_overlay {
  if [ $# -eq 0 ]; then
    echo "No dtbo to remove!"
    exit 1;
  fi
  
  for i
  do
    rm -fv /boot/overlays/$i.dtbo || exit 1;
	sed -i "/^dtoverlay="$i"$/d" ${CFG_PATH}
  done
  
  sed -i "/^dtoverlay=vc4-kms-v3d-pi4$/d" ${CFG_PATH}
}

function usage() {
  cat <<-__EOF__
    usage: sudo ./reTerminal.sh [ --autoremove | --install ] [ -h | --help ]
             default action is update lan7800 module.
             --install       used for update module
             --autoremove    used for automatic cleaning
             --help          show this help message
__EOF__
  exit 1
}

function install {
  check_kernel_headers
  build_modules mipi_dsi ltr30x lis3lv02d
  install_modules mipi_dsi ltr30x/als_ltr30x lis3lv02d/lis331dlh-i2c
  depmod -a

  install_overlay reTerminal
  
  cp -fv reTerminal/10-disp.conf /usr/share/X11/xorg.conf.d/ || exit 1;
  cp -fv reTerminal/plymouth/plymouthd.conf /etc/plymouth/ || exit 1;
  cp -rfv reTerminal/plymouth/seeed/ /usr/share/plymouth/themes/ || exit 1;

  echo "------------------------------------------------------"
  echo "Please reboot your device to apply all settings"
  echo "Enjoy!"
  echo "------------------------------------------------------"
}

function uninstall {
  clean_modules mipi_dsi ltr30x lis3lv02d
  uninstall_modules mipi_dsi als_ltr30x lis331dlh-i2c
  rm -fv overlays/rpi/.*.tmp
  rm -fv overlays/rpi/.*.cmd
  rm -fv overlays/rpi/*.dtbo
  depmod -a
  
  uninstall_overlay reTerminal
}

if [[ ! -z $1 ]]; then
  if [ $1 = "--autoremove" ]; then
    uninstall
  elif [ $1 = "--install" ]; then
    install
  else
    usage
  fi
else
  install
fi








