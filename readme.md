Build Status

 [![Build Status](http://gfnd.rcn-ee.org:8080/buildStatus/icon?job=beagleboard_overlays/master)](http://gfnd.rcn-ee.org:8080/job/beagleboard_overlays/job/master/)


Users:
------------

Install or Update bb-cape-overlays debian package (pre installed on: Debian Jessie/Stretch & Ubuntu Xenial images)

    sudo apt update ; sudo apt install bb-cape-overlays

Supported Linux Kernels:
------------

Pre-built kernels: (there are multiple options avaiable)

    cd /opt/scripts/tools
    git pull

Supported Linux Kernels: v4.19.x:
------------

v4.19.x-ti:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_19 --ti-channel

v4.19.x-ti + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_19 --ti-rt-channel

v4.19.x mainline:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_19 --bone-channel

v4.19.x mainline + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_19 --bone-rt-channel

Supported Linux Kernels: v4.14.x:
------------

v4.14.x-ti:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_14 --ti-channel

v4.14.x-ti + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_14 --ti-rt-channel

v4.14.x mainline:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_14 --bone-channel

v4.14.x mainline + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_14 --bone-rt-channel

Supported Linux Kernels: v4.9.x:
------------

v4.9.x-ti:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_9 --ti-channel

v4.9.x-ti + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_9 --ti-rt-channel

v4.9.x mainline:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_9 --bone-channel

v4.9.x mainline + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_9 --bone-rt-channel

Supported Linux Kernels: v4.4.x:
------------

v4.4.x-ti:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_4 --ti-channel

v4.4.x-ti + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_4 --ti-rt-channel

v4.4.x mainline:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_4 --bone-channel

v4.4.x mainline + Real Time:

    sudo /opt/scripts/tools/update_kernel.sh --lts-4_4 --bone-rt-channel

Developers:
------------

Step 1: Clone this repo:

    git clone https://github.com/beagleboard/bb.org-overlays
    cd ./bb.org-overlays

Step 2: Install *.dtbo:

    ./install.sh

Developers: Tested Versions of dtc:
------------

    v1.4.4
    v1.4.6
    v1.4.7
    v1.5.0

Known Broken: v1.4.5 (DO NOT USE)

U-Boot Overlays:
------------

https://elinux.org/Beagleboard:BeagleBoneBlack_Debian#U-Boot_Overlays

