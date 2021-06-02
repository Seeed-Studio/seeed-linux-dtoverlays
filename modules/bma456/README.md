GROVE Step Counter(BMA456)
=========================================

  After driver and configuration prepared well,
  the user could access ```/sys/bus/iio/devices/iio\:device<N>/XXX```,
  to get the sensor data results. If you need to write data, you must 
  use the root account by ```su root```.

***

Usage
-----

***example:***

  ```bash
  # check the device<N> name, make sure it's bma456
  # where <N> is a number specific to your system.
  $ cat /sys/bus/iio/devices/iio\:device0/name
  bma456
  
  # get the available low pass 3db frequency, in us.
  $ cat /sys/bus/iio/devices/iio\:device0/in_accel_filter_low_pass_3db_frequency_available
  1280000 640000 320000 160000 80000 40000 20000 10000 5000 2500 1250 625
  # set & get the low pass 3db frequency.
  echo 10000 > /sys/bus/iio/devices/iio\:device0/in_accel_filter_low_pass_3db_frequency
  cat /sys/bus/iio/devices/iio\:device0/in_accel_filter_low_pass_3db_frequency
  10000

  # get the temperature scale.
  $ cat /sys/bus/iio/devices/iio\:device0/in_temp_scale
  0.001000
  # get the temperature, in milli Celsius.
  $ cat /sys/bus/iio/devices/iio\:device0/in_temp_raw
  27000
  # the real temp = in_temp_scale * in_temp_raw

  # get the available accel scale.
  $ cat /sys/bus/iio/devices/iio\:device0/in_accel_scale_available
  0.000598 0.001197 0.002394 0.004789
  # set & get the accel scale.
  $ echo 0.000598 > /sys/bus/iio/devices/iio\:device0/in_accel_scale
  $ cat /sys/bus/iio/devices/iio\:device0/in_accel_scale
  0.000598

  # get the accel z raw, x/y is the same.
  $ cat /sys/bus/iio/devices/iio\:device0/in_accel_z_raw
  11630
  # the real gravity = in_accel_scale * in_accel_z_raw

  # set the stepcount enable, this will reset the stepcount.
  $ echo 1 > /sys/bus/iio/devices/iio\:device0/in_steps_en
  # swing the module several times, then get the stepcount
  $ cat /sys/bus/iio/devices/iio\:device0/in_steps_input
  12

  # any_motion and no_motion are the same method.
  # set any motion_enable
  $ echo 1 > /sys/bus/iio/devices/iio\:device0/any_motion_enable
  # set any motion sensitivity (1 bit = 0.48mG)
  $ echo 100 > /sys/bus/iio/devices/iio\:device0/any_motion_sensitivity
  # then you move the module, you can get the any_motion status
  cat /sys/bus/iio/devices/iio\:device0/any_motion
  1

  # single_tap and double_tap are the same method.
  # set the single_tap_enable
  echo 1 > single_tap_enable
  # set the single_tap_sensitivity, 0~7: MOST to LEAST.
  echo 0 > single_tap_sensitivity


pi@raspberrypi:/ $ evtest
No device specified, trying to scan all of /dev/input/event*
Not running as root, no devices may be available.
Available devices:
/dev/input/event0:      bma456
Select the device event number [0-0]: 0
Input driver version is 1.0.1
Input device ID: bus 0x18 vendor 0x1234 product 0x8888 version 0x100
Input device name: "bma456"
Supported events:
  Event type 0 (EV_SYN)
  Event type 1 (EV_KEY)
    Event code 330 (BTN_TOUCH)
Properties:
Testing ... (interrupt to exit)
Event: time 1607685236.585773, type 1 (EV_KEY), code 330 (BTN_TOUCH), value 1
Event: time 1607685236.585773, -------------- SYN_REPORT ------------
Event: time 1607685238.505782, type 1 (EV_KEY), code 330 (BTN_TOUCH), value 0
Event: time 1607685238.505782, type 1 (EV_KEY), code 330 (BTN_TOUCH), value 1
Event: time 1607685238.505782, -------------- SYN_REPORT ------------

  ```

  If you need customize the I2C-Port..., change it the device tree source.

