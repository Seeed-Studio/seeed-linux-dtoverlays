#!/bin/bash

# Check root
if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (use sudo)" 1>&2
  exit 1
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
  echo " Choose  *** y *** to revert the kernel to version $VER_HDR and continue."
  echo " Choose  *** N *** to exit without this driver support, by default."
  read -p "Would you like to proceed? (y/N)" -n 1 -r -s
  echo
  if ! [[ $REPLY =~ ^[Yy]$ ]]; then
    exit 1;
  fi

  apt-get -y --reinstall install raspberrypi-kernel-headers
}

# Build module
function build_modules {
  KBUILD=/lib/modules/`uname -r`/build
  MOD_PATH=`pwd`/modules

  if [ $# -eq 0 ]; then
    echo "No module to compile!"
	return 0;
  fi

  for i
  do
    make -C $KBUILD M=$MOD_PATH/$i || echo Build failed: $i;
  done
}

function clean_modules {
  KBUILD=/lib/modules/`uname -r`/build
  MOD_PATH=`pwd`/modules

  if [ $# -eq 0 ]; then
    echo "No module to compile!"
	return 0;
  fi

  for i
  do
    make -C $KBUILD M=$MOD_PATH/$i clean || echo Build failed: $i;
  done
}

# Install module
function install_modules {
  INS_PATH=/lib/modules/`uname -r`/extra
  MOD_PATH=`pwd`/modules
  
  if [ $# -eq 0 ]; then
    echo "No module to install!"
	return 0;
  fi
  
  for i
  do
    mkdir -p $INS_PATH;
    cp $MOD_PATH/$i/$i.ko $INS_PATH
	grep -q "^$i$" /etc/modules || \
	  echo $i >> /etc/modules
  done
}

function uninstall_modules {
  INS_PATH=/lib/modules/`uname -r`/extra
  MOD_PATH=`pwd`/modules
  
  if [ $# -eq 0 ]; then
    echo "No module to install!"
	return 0;
  fi
  
  for i
  do
    if [ -e $INS_PATH/$i.ko ]; then
      echo remove: $INS_PATH/$i.ko
      rm -f $INS_PATH/$i.ko
    fi

	sed -i '/^$i/d' /etc/modules
  done
}

# Add blacklist to cmdline
function add_blacklist {
  CLI_PATH=/boot/cmdline.txt

  if [ $# -eq 0 ]; then
    echo "No module to add to blacklist!"
	return 0;
  fi

  CMDLINE=$(cat $CLI_PATH | sed 's/\binitcall_blacklist=\S*\b *//g')
#  echo $CMDLINE
  
  BLACKLIST=$(grep -o "\binitcall_blacklist=\S*\b" $CLI_PATH)
#  echo $BLACKLIST
  
  for i
  do
     if [ $(echo $BLACKLIST | grep -c "$i") -eq 0 ]; then
	   if [ ${#BLACKLIST} -eq 0 ]; then
	     BLACKLIST="initcall_blacklist=";
	   else
	     BLACKLIST="$BLACKLIST,";
	   fi
	   BLACKLIST="$BLACKLIST$i";
	 fi
  done
  
  CMDLINE="$CMDLINE $BLACKLIST"
  echo $CMDLINE > /boot/cmdline.txt
}

function remove_blacklist {
  CLI_PATH=/boot/cmdline.txt
  
  if [ $# -eq 0 ]; then
    echo "No module to add to blacklist!"
	return 0;
  fi
  
  CMDLINE=$(cat $CLI_PATH | sed 's/\binitcall_blacklist=\S*\b *//g')
  
  echo $CMDLINE > /boot/cmdline.txt
}

function usage() {
  cat <<-__EOF__
    usage: sudo ./cm4_lan7800.sh [ --autoremove ] [ -h | --help ]
             default action is update lan7800 module.
             --autoremove    used for automatic cleaning.
             --help          show this help message
__EOF__
  exit 1
}

if [[ ! -z $1 ]]; then
  if [[ $1 = "-h" || $1 = "--help" ]]; then
    usage
  else [ $1 = "--autoremove" ]
    clean_modules lan7800
	uninstall_modules lan7800
	remove_blacklist lan78xx_driver_init
  fi
else
  check_kernel_headers
  build_modules lan7800
  install_modules lan7800
  add_blacklist lan78xx_driver_init
fi








