# Run ReSpeaker 2-Mics Pi HAT/WM8960 on Jetson Nano

For Jetson source R32.4.2 or JetPack Image 4.4

### 1. Clone repo
```shell
	cd; git clone https://github.com/Seeed-Studio/seeed-linux-dtoverlays
	cd ~/seeed-linux-dtoverlays
```

### 2. Build dtbo & driver
```shell
	export CUSTOM_MOD_LIST="jtsn-wm8960"
	make all_jetsonnano
```

### 3. Install driver
```shell
	sudo -E make install_jetsonnano
```

### 4. Install dtbo
```shell
	sudo cp overlays/jetsonnano/jetson-seeed-2mic-wm8960.dtbo /boot
	sudo /opt/nvidia/jetson-io/config-by-hardware.py -n "Seeed Voice Card 2MIC"
```

### 5. Reboot
```shell
	sudo reboot
```

### 6. Restore ALSA mixer/widgets setting
```shell
	# must wait a momemnt the time sound card busy after login
	cd ~/seeed-linux-dtoverlays
	alsactl -f extras/wm8960_asound.state-jetson-nano restore 1
```

### 7. Capture & Playback
```shell
	arecord -D hw:1,0 -f S32_LE -r 48000 -c 2 | aplay -D hw:1,0 -f S32_LE -r 48000 -c 2
```
