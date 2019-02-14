# CAN-HAT


The drivers of [Raspberry pi  CAN HAT](https://www.seeedstudio.com/ReSpeaker-2-Mics-Pi-HAT-p-2874.html) for Raspberry Pi.

### Install CAN-HAT
Get the CAN-HAT  source code. and install all linux kernel drivers
```bash
git clone https://github.com/seeed-Studio/pi-hats
cd pi-hats/CAN-HAT
git checkout dev
sudo ./install.sh 
sudo reboot
```

## Raspberry pi  CAN HAT

[![](https://github.com/SeeedDocument/MIC_HATv1.0_for_raspberrypi/blob/master/img/mic_hatv1.0.png?raw=true)](https://www.seeedstudio.com/ReSpeaker-2-Mics-Pi-HAT-p-2874.html)


Check the kernel log to see if MCP2517 was initialized successfully.You will also see can0 and can1 appear in the list of ifconfig results

```bash
pi@raspberrypi:~ $ dmesg | grep spi
[    3.725586] mcp25xxfd spi0.0 can0: MCP2517 successfully initialized.
[    3.757376] mcp25xxfd spi1.0 can1: MCP2517 successfully initialized.

pi@raspberrypi:~/pi-hats/CAN-HAT $ ifconfig -a
can0: flags=128<NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

can1: flags=128<NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

eth0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether b8:27:eb:c7:ed:4f  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.250.42  netmask 255.255.255.0  broadcast 192.168.250.255
        inet6 fe80::3842:7323:7c0d:f6d2  prefixlen 64  scopeid 0x20<link>
        ether b8:27:eb:92:b8:1a  txqueuelen 1000  (Ethernet)
        RX packets 2654  bytes 249303 (243.4 KiB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 4433  bytes 4765896 (4.5 MiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

```




#### Next step
Set the can fd protocol, and the dbitrate can be set to 8M speed. [Refer to the kernel documentation for more usage](https://www.kernel.org/doc/Documentation/networking/can.txt)

```bash
sudo ip link set can0 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
sudo ip link set can1 up type can bitrate 1000000   dbitrate 8000000 restart-ms 1000 berr-reporting on fd on

sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536
```

The hardware is wired to can0 and can1 interface.

0_L  <===> 1_L

0_H  <===> 1_H

Open two terminal windows and enter the following commands in the Windows to test can fd protocol.
```bash
#send data
cangen can0 -mv 
```

```bash
#dump data
candump can0 
```


### uninstall CAN-HAT

```
pi@raspberrypi:~/pi-hats/CAN-HAT $ sudo ./uninstall.sh 
...
------------------------------------------------------
Please reboot your raspberry pi to apply all settings
Thank you!
------------------------------------------------------
```


Enjoy !
