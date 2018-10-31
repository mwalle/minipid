# minipid - an attiny based heating controller

minipid is an open source hardware and software PI controller. It's design
goal is to be cheap and easy to build. It was originally developed for the
Rancilio Siliva.

## Warning

This project uses dangerous mains voltage. Be careful!

## Features

 * cheap
 * configurable parameters, like target temperature, Kp and Ki and controlling range
 * auto off
 * built in self test
 * configuration interface on the serial line
 * display support
 * ISP support
 * fast PI controller
 * emergency off
 * error blinking, if no display is connected
 * easy to use, easy to built
 * dump parameters like temperature and output value to serial console
 * automatically detects connected display
 * open hardware and open source software

## License

The firmware is licensed under the GNU Public License version 2 and the
hardware is licensed under CERN OHL v.1.2. See `COPYING.firmware` and
`COPYING.hardware` for the complete license.

## Hardware Errata

 - [ ] SDA and SCL pull-downs are missing
