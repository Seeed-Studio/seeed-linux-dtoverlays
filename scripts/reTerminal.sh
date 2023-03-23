#!/bin/bash

# module version
ver="0.1"

# we create a dir with this version to ensure that 'dkms remove' won't delete
# the sources during kernel updates
marker="0.0.0"

FORCE_KERNEL="1.20210303-1"

uname_r=$(uname -r)
arch_r=$(dpkg --print-architecture)

# Common path
SRC_PATH=/usr/src
MOD_PATH=`pwd`/modules
RES_PATH=`pwd`/extras/reTerminal/resources

# RPI
CFG_PATH=/boot/config.txt
CLI_PATH=/boot/cmdline.txt
OVERLAY_DIR=/boot/overlays
# Ubuntu
[ -f /boot/firmware/config.txt ] && CFG_PATH=/boot/firmware/config.txt
[ -f /boot/firmware/cmdline.txt ] && CLI_PATH=/boot/firmware/cmdline.txt
[ -d /boot/firmware/overlays ] && OVERLAY_DIR=/boot/firmware/overlays

_VER_RUN=""
function get_kernel_version() {
  local ZIMAGE IMG_OFFSET

  [ -z "$_VER_RUN" ] && {
    ZIMAGE=/boot/kernel7l.img
    if [ $arch_r == "arm64" ]; then
      ZIMAGE=/boot/kernel8.img
    fi
    [ -f /boot/firmware/vmlinuz ] && ZIMAGE=/boot/firmware/vmlinuz
    IMG_OFFSET=$(LC_ALL=C grep -abo $'\x1f\x8b\x08\x00' $ZIMAGE | head -n 1 | cut -d ':' -f 1)
    _VER_RUN=$(dd if=$ZIMAGE obs=64K ibs=4 skip=$(( IMG_OFFSET / 4)) 2>/dev/null | zcat | grep -a -m1 "Linux version" | strings | awk '{ print $3; }' | grep "[0-9]")
  }
  echo "$_VER_RUN"
  
  return 0
}

# Check headers
function check_kernel_headers() {
  VER_RUN=$(get_kernel_version)
  return 1;

  if [ -d "/lib/modules/${VER_RUN}/build" ]; then
    echo KBUILD: "/lib/modules/${VER_RUN}/build"
	return 0;
  fi

  echo " !!! Your kernel version is $VER_RUN"
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

function download_install_debpkg() {
  local prefix name r pkg status _name
  prefix=$1
  name=$2
  pkg=${name%%_*}

  status=$(dpkg -l $pkg | tail -1)
  _name=$(  echo "$status" | awk '{ printf "%s_%s_%s", $2, $3, $4; }')
  status=$(echo "$status" | awk '{ printf "%s", $1; }')

  if [ "X$status" == "Xii" -a "X${name%.deb}" == "X$_name" ]; then
    echo "debian package $name already installed."
    return 0
  fi

  for (( i = 0; i < 3; i++ )); do
    wget $prefix$name -O /tmp/$name && break
  done

  dpkg -i /tmp/$name; r=$?
  rm -f /tmp/$name

  return $r
}

function install_kernel() {
  local _url _prefix

  # Instead of retrieving the lastest kernel & headers
  [ "X$FORCE_KERNEL" == "X" ] && {
    # Raspbian kernel packages
    apt-get -y --force-yes install raspberrypi-kernel-headers raspberrypi-kernel || {
      # Ubuntu kernel packages
      apt-get -y install linux-raspi linux-headers-raspi linux-image-raspi
    }
  } || {
    # We would like to a fixed version
    KERN_NAME=raspberrypi-kernel_${FORCE_KERNEL}_${arch_r}.deb
    HDR_NAME=raspberrypi-kernel-headers_${FORCE_KERNEL}_${arch_r}.deb
    _url=$(apt-get download --print-uris raspberrypi-kernel | sed -nre "s/'([^']+)'.*$/\1/g;p")
    _prefix=$(echo $_url | sed -nre 's/^(.*)raspberrypi-kernel_.*$/\1/g;p')

    download_install_debpkg "$_prefix" "$KERN_NAME" && {
      download_install_debpkg "$_prefix" "$HDR_NAME"
    } || {
      echo "Error: Install kernel or header failed"
      exit 2
    }
  }
}

# Install module
function install_modules {
  if [ $# -eq 0 ]; then
    echo "No module to install!"
    exit 1;
  fi

  # locate currently installed kernels (may be different to running kernel if
  # it's just been updated)
  kernel=$(get_kernel_version)

  for mod
  do
    target=$SRC_PATH/$mod-$ver
    mkdir -p $target
	cp -a $MOD_PATH/$mod/* $target/

	dkms build -k ${kernel} -m $mod -v $ver && {
      dkms install --force -k ${kernel} -m $mod -v $ver
    } || {
      echo "Can't compile with this kernel, aborting"
      echo "Please try to compile with the option --compat-kernel"
      cat /var/lib/dkms/$mod/$ver/build/make.log
      exit 1
    }

	mkdir -p /var/lib/dkms/$mod/$ver/$marker
  done
}

function uninstall_modules { 
  if [ $# -eq 0 ]; then
    echo "No module to uninstall!"
    exit 1;
  fi

  for mod
  do
    if [[ -d /var/lib/dkms/$mod/$ver/$marker ]]; then
      rmdir /var/lib/dkms/$mod/$ver/$marker
    fi

    if [[ -e $SRC_PATH/$mod-$ver || -e /var/lib/dkms/$mod/$ver ]]; then
      dkms remove --force -m $mod -v $ver --all
      rm -rf $SRC_PATH/$mod-$ver
    fi
  done
}

# set dtparam=$1=$2
function set_config_dtparam {
  if [ "`grep ".*dtparam=$1=.*$" ${CFG_PATH}`" ]; then
    # item exist
    sed -i "s/.*dtparam=$1=.*$/dtparam=$1=$2/g" ${CFG_PATH}
  else
    # not exist, apply new item
    echo "dtparam=$1=$2" >> $CFG_PATH 
  fi
}

# remove dtparam=$1=$2
function remove_config_dtparam {
  sed -i "/^dtparam=$1=$2$/d" ${CFG_PATH}
}

# set dtoverlay=$1
function set_config_dtoverlay {
  if [ "`grep ".*dtoverlay=$1$" ${CFG_PATH}`" ]; then
    # item exist
    sed -i "s/.*dtoverlay=$1$/dtoverlay=$1/g" ${CFG_PATH}
  else
    # not exist, apply new item
    echo "dtoverlay=$1" >> $CFG_PATH 
  fi
}

# remove dtoverlay=$1
function remove_config_dtoverlay {
  sed -i "/^dtoverlay=$1$/d" ${CFG_PATH}
}

# commit config value
function commit_config_value {
    sed -i "s/^$1$/#&/g" ${CFG_PATH}
}

# set $1=$2
function set_config_value {
  grep -q "^$1=$2$" $CFG_PATH || \
    echo "$1=$2" >> $CFG_PATH
}

# remove $1=$2
function remove_config_value {
  sed -i "/^$1=$2$/d" ${CFG_PATH}
}

# apply new value on cmdline
function set_cmdline_value {
  grep -q "\b$1\b" $CLI_PATH || \
    sed -i "s/$/& $1/g" $CLI_PATH
}

# remove value on cmdline
function remove_cmdline_value {
  sed -i "s/ *\b$1\b//g" $CLI_PATH
}

function install_overlay_DM {
  # cmdline.txt
  remove_cmdline_value "console=tty0"

  set_cmdline_value "logo.nologo"
  set_cmdline_value "vt.global_cursor_default=0"
  set_cmdline_value "console=tty3"
  set_cmdline_value "loglevel=0"

  # config.txt
  set_config_dtparam "i2c_arm" "on"
  set_config_dtparam "i2c_vc" "on"
  set_config_dtparam "i2s" "on"
  set_config_dtparam "spi" "on"

  set_config_value "enable_uart" "1"
  set_config_value "dtparam" "ant2"
  set_config_value "disable_splash" "1"
  set_config_value "ignore_lcd" "1"

  set_config_dtoverlay "dwc2,dr_mode=host"
  set_config_dtoverlay "vc4-kms-v3d-pi4"
  set_config_dtoverlay "i2c1,pins_2_3"
  set_config_dtoverlay "i2c3,pins_4_5"
  set_config_dtoverlay "imx219,cam0"

  make overlays/rpi/reTerminal-plus-overlay.dtbo || exit 1;
  cp -fv overlays/rpi/reTerminal-plus-overlay.dtbo $OVERLAY_DIR/reTerminal-plus.dtbo || exit 1;

  set_config_dtoverlay "reTerminal-plus"

  # commit old parameters
  commit_config_value "gpio=13=pu"
  commit_config_value "dtoverlay=reTerminal,tp_rotate=1"
  commit_config_value "dtoverlay=reTerminal-bridge"
}

function uninstall_overlay_DM {
  # cmdline.txt
  remove_cmdline_value "logo.nologo"
  remove_cmdline_value "vt.global_cursor_default=0"
  remove_cmdline_value "console=tty3"
  remove_cmdline_value "loglevel=0"

  # config.txt
  remove_config_dtparam "i2c_arm" "on"
  remove_config_dtparam "i2c_vc" "on"
  remove_config_dtparam "i2s" "on"
  remove_config_dtparam "spi" "on"

  remove_config_value "enable_uart" "1"
  remove_config_value "dtparam" "ant2"
  remove_config_value "disable_splash" "1"
  remove_config_value "ignore_lcd" "1"

  remove_config_dtoverlay "dwc2,dr_mode=host"
  remove_config_dtoverlay "vc4-kms-v3d-pi4"
  remove_config_dtoverlay "i2c1,pins_2_3"
  remove_config_dtoverlay "i2c3,pins_4_5"
  remove_config_dtoverlay "imx219,cam0"

  rm -fv $OVERLAY_DIR/reTerminal-plus.dtbo || exit 1;
  remove_config_dtoverlay "reTerminal-plus"

  rm -fv overlays/rpi/.*.tmp
  rm -fv overlays/rpi/.*.cmd
  rm -fv overlays/rpi/*.dtbo
}

# Overlay
function install_overlay {
  if [ $# -eq 0 ]; then
    echo "No dtbo to install!"
    exit 1;
  fi

  # cmdline.txt
  CMDLINE=$(cat $CLI_PATH)
  CMDLINE=$(echo $CMDLINE | sed 's/ *\bconsole=tty0\b//g')
  grep -q "\blogo.nologo\b" $CLI_PATH || \
    CMDLINE="$CMDLINE logo.nologo"
  grep -q "\bvt.global_cursor_default=0\b" $CLI_PATH || \
    CMDLINE="$CMDLINE vt.global_cursor_default=0"
  grep -q "\bconsole=tty3\b" $CLI_PATH || \
    CMDLINE="$CMDLINE console=tty3"
  grep -q "\bloglevel=0\b" $CLI_PATH || \
    CMDLINE="$CMDLINE loglevel=0"
  echo $CMDLINE > $CLI_PATH

  # config.txt
  sed -i "s/.*dtparam=i2c_arm=.*$/dtparam=i2c_arm=on/g" ${CFG_PATH}

  grep -q "^enable_uart=1$" $CFG_PATH || \
    echo "enable_uart=1" >> $CFG_PATH
  grep -q "^dtoverlay=dwc2,dr_mode=host$" $CFG_PATH || \
    echo "dtoverlay=dwc2,dr_mode=host" >> $CFG_PATH
  grep -q "^dtparam=ant2$" $CFG_PATH || \
    echo "dtparam=ant2" >> $CFG_PATH
  grep -q "^disable_splash=1$" $CFG_PATH || \
    echo "disable_splash=1" >> $CFG_PATH

  grep -q "^ignore_lcd=1$" $CFG_PATH || \
    echo "ignore_lcd=1" >> $CFG_PATH
  grep -q "^dtoverlay=vc4-kms-v3d-pi4$" $CFG_PATH || \
    echo "dtoverlay=vc4-kms-v3d-pi4" >> $CFG_PATH
  grep -q "^dtoverlay=i2c3,pins_4_5$" $CFG_PATH || \
    echo "dtoverlay=i2c3,pins_4_5" >> $CFG_PATH
  grep -q "^gpio=13=pu$" $CFG_PATH || \
    echo "gpio=13=pu" >> $CFG_PATH

  for i
  do
    make overlays/rpi/$i-overlay.dtbo || exit 1;
    cp -fv overlays/rpi/$i-overlay.dtbo $OVERLAY_DIR/$i.dtbo || exit 1;

	grep -q "^dtoverlay=$i$" $CFG_PATH || \
	  echo "dtoverlay=$i" >> $CFG_PATH
  done
}

function uninstall_overlay {
  if [ $# -eq 0 ]; then
    echo "No dtbo to remove!"
    exit 1;
  fi

  # cmdline.txt
  CMDLINE=$(cat $CLI_PATH)
  CMDLINE=$(echo $CMDLINE | sed 's/ *\blogo.nologo\b//g')
  CMDLINE=$(echo $CMDLINE | sed 's/ *\bvt.global_cursor_default=0\b//g')
  CMDLINE=$(echo $CMDLINE | sed 's/ *\bconsole=tty3\b//g')
  CMDLINE=$(echo $CMDLINE | sed 's/ *\bloglevel=0\b//g')
  echo $CMDLINE > $CLI_PATH

  # config.txt
  sed -i "/^disable_splash=1$/d" ${CFG_PATH}
  sed -i "/^ignore_lcd=1$/d" ${CFG_PATH}
  sed -i "/^dtoverlay=vc4-kms-v3d-pi4$/d" ${CFG_PATH}
  sed -i "/^dtoverlay=i2c3,pins_4_5$/d" ${CFG_PATH}
  sed -i "/^gpio=13=pu$/d" ${CFG_PATH}

  for i
  do
    rm -fv $OVERLAY_DIR/$i.dtbo || exit 1;
	sed -i "/^dtoverlay="$i"$/d" ${CFG_PATH}
  done

  rm -fv overlays/rpi/.*.tmp
  rm -fv overlays/rpi/.*.cmd
  rm -fv overlays/rpi/*.dtbo
}

function setup_overlay {
  sed -i "/^dtoverlay=$1$/s//dtoverlay=$1,$2/" ${CFG_PATH}
}

#NOTICE: this function must be used
# before the uninstall_overlay
function unsetup_overlay {
  sed -i "/^dtoverlay=$1,$2$/s//dtoverlay=$1/" ${CFG_PATH}
}

function usage() {
  cat <<-__EOF__
    usage: sudo ./scripts/reTerminal.sh [ --autoremove | --install ] [ -h | --help ] [ --device <type>]
             default action is update kernel & headers to latest version.
             --compat-kernel uses an older kernel but ensures that the driver can work.
             --keep-kernel   don't change/update the system kernel, maybe install
                             coressponding kernel headers.
             --autoremove    used for automatic cleaning
             --device <type>   specified device type. 
                               if device is reTerminal-plus, type=reTerminal-plus
                               if device is reTerminal, type=reTerminal
                               the default device type value is reTerminal
             --help          show this help message
__EOF__
  exit 1
}

function install {
  if [ "$device" = "reTerminal" ]; then
    install_modules mipi_dsi ltr30x lis3lv02d bq24179_charger
    install_overlay reTerminal reTerminal-bridge
    setup_overlay reTerminal tp_rotate=1
  elif [ "$device" = "reTerminal-plus" ]; then
    install_modules ltr30x ili9881d
    install_overlay_DM
  fi
  # display
 if ! [[ -d "/usr/share/plymouth/themes/" && -d "/usr/share/X11/xorg.conf.d/" && -d "/etc/plymouth/" ]];
  then
    mkdir -p /usr/share/plymouth/themes/ \
    /usr/share/X11/xorg.conf.d/ \
    /etc/plymouth/
  fi
  cp -rfv ${RES_PATH}/plymouth/seeed/ /usr/share/plymouth/themes/ || exit 1;
  cp -fv ${RES_PATH}/10-disp.conf /usr/share/X11/xorg.conf.d/ || exit 1;
  cp -fv ${RES_PATH}/plymouth/plymouthd.conf /etc/plymouth/ || exit 1;

  # audio
  if [ "$device" = "reTerminal" ]; then
    if [ -f "/var/lib/alsa/asound.state" ]; then
      cp /var/lib/alsa/asound.state /var/lib/alsa/asound.state.bak
    fi
    if [ -f "/etc/asound.conf" ]; then
      cp /etc/asound.conf /etc/asound.conf.bak
    fi
    cp ${MOD_PATH}/seeed-voicecard/wm8960_asound.state /var/lib/alsa/asound.state 
    cp ${MOD_PATH}/seeed-voicecard/asound_2mic.conf /etc/asound.conf
    alsactl -L restore
  fi

  echo "------------------------------------------------------"
  echo "Please reboot your device to apply all settings"
  echo "Enjoy!"
  echo "------------------------------------------------------"
}

function uninstall {
  if [ "$device" = "reTerminal" ]; then
    uninstall_modules mipi_dsi ltr30x lis3lv02d bq24179_charger
    unsetup_overlay reTerminal tp_rotate=1
    uninstall_overlay reTerminal reTerminal-bridge
  elif [ "$device" = "reTerminal-plus" ]; then
    uninstall_modules ili9881d ltr30x
    uninstall_overlay_DM
  fi
}


# Check root
if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (use sudo)" 1>&2
  exit 1;
fi

compat_kernel=
keep_kernel=
auto_remove=
device="reTerminal"
# parse commandline options
while [ ! -z "$1" ] ; do
  case $1 in
  -h|--help)
    usage
    ;;
  --compat-kernel)
    compat_kernel=Y
    ;;
  --keep-kernel)
    keep_kernel=Y
    ;;
  --autoremove)
    auto_remove=Y
    ;;
  --device)
    shift
    device=$1
	;;
  esac
  shift
done

if [ "$device" != "reTerminal" ] && [ "$device" != "reTerminal-plus" ]; then
  echo "Invalid device type. the type should be reTerminal or reTerminal-plus" 1>&2
  exit 1;
fi

if [ "X$auto_remove" != "X" ]; then
  uninstall
  echo "Auto remove: finished!"
  exit 0
fi

# Check if enough space on /boot volume
boot_line=$(df -BM | grep /boot | head -n 1)
MIN_BOOT_SPC=25 # MegaBytes
if [ "x${boot_line}" = "x" ]; then
  echo "Warning: /boot volume not found .."
else
  boot_space=$(echo $boot_line | awk '{print $4;}')
  free_space=$(echo "${boot_space%?}")
  unit="${boot_space: -1}"
  if [[ "$unit" != "M" ]]; then
    echo "Warning: /boot volume not found .."
  elif [ "$free_space" -lt "$MIN_BOOT_SPC" ]; then
    echo "Error: Not enough space left ($boot_space) on /boot"
    echo "       at least $MIN_BOOT_SPC MB required"
    exit 1
  fi
fi

# update and install required packages
which apt &>/dev/null; r=$?
if [[ $r -eq 0 ]]; then
  echo -e "\n### Install required tool packages"
  apt-get update -y
  apt-get -y install dkms --no-install-recommends
fi

if [ "X$keep_kernel" != "X" ]; then
  FORCE_KERNEL=$(dpkg -s raspberrypi-kernel | awk '/^Version:/{printf "%s\n",$2;}')
  echo -e "\n### Keep current system kernel not to change"
elif [ "X$compat_kernel" != "X" ]; then
  echo -e "\n### Will compile with a compatible kernel..."
else
  FORCE_KERNEL=""
  echo -e "\n### Will compile with the latest kernel..."
fi
[ "X$FORCE_KERNEL" != "X" ] && {
  echo -e "The kernel & headers use package version: $FORCE_KERNEL"
}

echo -e "\n### Uninstall previous dkms module"
uninstall

if [[ $r -eq 0 ]]; then
  echo -e "\n### Install required kernel package"
  install_kernel
  check_kernel_headers
fi

install










