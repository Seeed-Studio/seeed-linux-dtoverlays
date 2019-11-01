#!/bin/bash

FORCE_KERNEL="1.20190925+1-1"

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (use sudo)" 1>&2
   exit 1
fi

is_Raspberry=$(cat /proc/device-tree/model | awk  '{print $1}')
if [ "x${is_Raspberry}" != "xRaspberry" ] ; then
  echo "Sorry, this drivers only works on raspberry pi"
  exit 1
fi

ver="0.1"

# we create a dir with this version to ensure that 'dkms remove' won't delete
# the sources during kernel updates
marker="0.0.0"

_VER_RUN=
function get_kernel_version() {
  local ZIMAGE IMG_OFFSET

  _VER_RUN=""
  [ -z "$_VER_RUN" ] && {
    ZIMAGE=/boot/kernel.img
    IMG_OFFSET=$(LC_ALL=C grep -abo $'\x1f\x8b\x08\x00' $ZIMAGE | head -n 1 | cut -d ':' -f 1)
    _VER_RUN=$(dd if=$ZIMAGE obs=64K ibs=4 skip=$(( IMG_OFFSET / 4)) | zcat | grep -a -m1 "Linux version" | strings | awk '{ print $3; }')
  }
  echo "$_VER_RUN"
  return 0
}

function check_kernel_headers() {
  VER_RUN=$(get_kernel_version)
  VER_HDR=$(dpkg -L raspberrypi-kernel-headers | egrep -m1 "/lib/modules/[^-]+/build" | awk -F'/' '{ print $4; }')
  [ "X$VER_RUN" == "X$VER_HDR" ] && {
    return 0
  }

  # echo RUN=$VER_RUN HDR=$VER_HDR
  echo " !!! Your kernel version is $VER_RUN"
  echo "     Not found *** coressponding *** kernel headers with apt-get."
  echo "     This may occur if you have ran 'rpi-update'."
  echo " Choose  *** y *** will revert the kernel to version $VER_HDR then continue."
  echo " Choose  *** N *** will exit without this driver support, by default."
  read -p "Would you like to proceed? (y/N)" -n 1 -r -s
  echo
  if ! [[ $REPLY =~ ^[Yy]$ ]]; then
    exit 1;
  fi

  apt-get -y --reinstall install raspberrypi-kernel
}

function download_install_debpkg() {
  local prefix name r
  prefix=$1
  name=$2

  for (( i = 0; i < 3; i++ )); do
    wget $prefix$name -O /tmp/$name && break
  done
  dpkg -i /tmp/$name; r=$?
  rm -f /tmp/$name
  return $r
}

function install_kernel() {
  local _url _prefix

   # Instead of retriving the lastest kernel & headers
  [ "X$FORCE_KERNEL" == "X" ] && {
    apt-get -y --force-yes install raspberrypi-kernel-headers raspberrypi-kernel
  } || {
    # We would like to a fixed version
    KERN_NAME=raspberrypi-kernel_${FORCE_KERNEL}_armhf.deb
    HDR_NAME=raspberrypi-kernel-headers_${FORCE_KERNEL}_armhf.deb
    _url=$(apt-get download --print-uris raspberrypi-kernel | sed -nre "s/'([^']+)'.*$/\1/g;p")
    _prefix=$(echo $_url | sed -nre 's/^(.*)raspberrypi-kernel_.*$/\1/g;p')

    download_install_debpkg "$_prefix" "$KERN_NAME"
    download_install_debpkg "$_prefix" "$HDR_NAME"
  }
}

# update and install required packages
which apt &>/dev/null
if [[ $? -eq 0 ]]; then
  apt update -y
  echo "____________________________________"
  apt-get -y install dkms git can-utils
  echo "____________________________________"
  install_kernel
  # rpi-update checker
  check_kernel_headers
fi

# Arch Linux
which pacman &>/dev/null
if [[ $? -eq 0 ]]; then
  pacman -Syu --needed git gcc automake make dkms linux-raspberrypi-headers can-utils
fi

# locate currently installed kernels (may be different to running kernel if
# it's just been updated)
base_ver=$(get_kernel_version)
base_ver=${base_ver%%[-+]*}
kernels="${base_ver}+ ${base_ver}-v7+ ${base_ver}-v7l+"

function install_module {
  local _i

  src=$1
  mod=$2

  if [[ -d /var/lib/dkms/$mod/$ver/$marker ]]; then
    rmdir /var/lib/dkms/$mod/$ver/$marker
  fi

  if [[ -e /usr/src/$mod-$ver || -e /var/lib/dkms/$mod/$ver ]]; then
    dkms remove --force -m $mod -v $ver --all
    rm -rf /usr/src/$mod-$ver
  fi

  mkdir -p /usr/src/$mod-$ver
  cp -a $src/* /usr/src/$mod-$ver/

  dkms add -m $mod -v $ver
  for _i in $kernels; do
    dkms build -k $_i -m $mod -v $ver && {
      dkms install --force -k $_i -m $mod -v $ver
    }
  done

  mkdir -p /var/lib/dkms/$mod/$ver/$marker
}

install_module "./" "mcp2517fd"


# install dtbos
cp 2xMCP2517FD.dtbo /boot/overlays



#set kernel moduels
grep -q "^mcp25xxfd-can$" /etc/modules || \
  echo "mcp25xxfd-can" >> /etc/modules

#set dtoverlays
grep -q "^dtoverlay=2xMCP2517FD$" /boot/config.txt || \
  echo "dtoverlay=2xMCP2517FD" >> /boot/config.txt


#install config files
cp 80-can.rules /etc/udev/rules.d/


echo "------------------------------------------------------"
echo "Please reboot your raspberry pi to apply all settings"
echo "Enjoy!"
echo "------------------------------------------------------"
