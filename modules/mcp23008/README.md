EXT-GPIO for LED & Input devices
=======================

  After driver and configuration prepared well, the user could access the following sysfs nodes：

  ***leds:***

  ```/sys/class/leds/usr_led<N>/brightness```

  ***keys:***
  
  ```/dev/input/event<N>```

  For more description of ```/dev/input/event<N>```, refer to
  [input](https://github.com/raspberrypi/linux/blob/rpi-5.10.y/Documentation/input/input.rst).

Usage
-----

  ***leds:***
   ```bash
  # use the root account
  su root
  # turn on the led
  echo 255 > /sys/class/leds/usr_led<N>/brightness
  # turn off the led
  echo 0 > /sys/class/leds/usr_led<N>/brightness
  # exit the root account
  exit
  ```

  ***keys:***

  ```bash
  sudo apt-get install evtest

  evtest
  # now select the number <N> of event<N>
  # then pressing the button(change Grove device output level),
  # see the evtest output
  ```

  If you need customize the GPIO-Port/key-code/label..., refer to device tree configuration
  [grove,button.txt](../../doc/devicetree/bindings/grove,button.txt)
