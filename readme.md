# PiInTheSky compatible receiver
A port of Dave Ackerman's [lora-gateway](https://github.com/PiInTheSky/lora-gateway) [Habitat](http://habhub.org/) interface to a PJRC Teensy-LC.  Designed to be included in a battery powered portable tracking antenna for balloon recovery, this receiver was designed to display telemetry from my makerspace's high-altitude balloon flight during the Aug 21, 2017 Eclipse on an old Sparkfun intelligent bitmap LCD.

While it's unlikely this code will be useful as-is for others, it is hoped it may be helpful as the basis of a portable tracker that can be used with PiInTheSky payloads.

![Teensy-LC PITS Tracker](https://raw.githubusercontent.com/danjulio/PiInTheSky-tracker/master/pictures/lora_rx_2b.png)

## Hardware

![Hardware Components](https://raw.githubusercontent.com/danjulio/PiInTheSky-tracker/tree/master/pictures/lora_rx_2.png)

Hardware is comprised of the following:
1. PJRC Teensy-LC. Any of the Teensy 3 family should work with minimal change.
2. RMF95 LoRa radio module (based on SX1276 chip).  I used an Adafruit Featherwing breakout board but any should work. 
3. 6 dBi Yagi antenna.  Selected as a trade-off between directionality, weight and gain.  A simple wire soldered to the radio module's antenna connection could work.  The antenna should match the LoRa frequency.
4. (Now obsolete) Sparkfun 128x64 pixel serial LCD module with built-in 8x6 pixel font.  The sofware assumes a display of 8 lines by 21 characters.  As described below it should be portable to any display that can manage this resolution.  I've included a version of the original Sparkfun Arduino library ported to use the hardware Serial1 port on the Teensy-LC (simply changed "Serial" to "Serial1").

Wiring is fairly simple and is described in the comments at the top of the teensy_gateway.ino file.  I kept some support for two radios in the code I ported from Dave's lora-gateway although it is not used currently.

## Software

![Typical Display](https://raw.githubusercontent.com/danjulio/PiInTheSky-tracker/tree/master/pictures/lora_rx_2a.png)

The software is based around a heavily hacked, ported version of Dave's gateway.c file with the data displayed on the LCD (and packet and telemetry data echoed to the Teensy's USB Serial).  I added a new LoRa configuration (#8) to Dave's PiInTheSky code I am using for 915 MHz operation in the USA.  It achieves about 1200 bps using 62.5 kHz bandwidth, 4:6EC and SF8.

The code detects both telemetry (including GPS) packets and SSDV packets.  It processes the telemetry packets but ignores (currently) the SSDV packets.  I would like eventually to echo them in some form to the Teensy USB Serial so that a program running on a PC could display them and telemetry information.

Telemetry data including GPS, altitude and temperature is displayed on the LCD along with information from the radio showing signal strength and current tuned frequency.

Items that existed in the PiInTheSky configuration files are set as constants at the top of the main sketch file.


## Display Note

I used an old Sparkfun display I had laying around but it's likely that other implementations would use a more modern display.  For that reason and the fact that the display was incredibly slow at updating caused me to separate out the telemetry processing from display.  The program writes character data into a 8 line by 21 character text buffer.  The display code then copies that to a temporary buffer and scans through it looking for changed text to update on the display.  This decouples processing of telemetry from display update.

