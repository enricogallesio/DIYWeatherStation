DIYWeatherStation
=================

PROJECT: DIY Weather Station
created on Dec. 2013
by IZ1ZCK (Enrico Gallesio)

Weather graphs (when available):
https://thingspeak.com/channels/8836

DESCRIPTION:
Sketch for Arduino Fio to operate a DIY Weather Station based on several 
sensor modules, store weather data on SD Card and publish them 
on any "Internet of Things" websites or via APRS Amateur Radio network.
XBee Series1 modules, 1 XBee Explorer and 1 Raspberry Pi with internet 
connection that runs a Python script are currently needed in this configuration
to allow wireless operation and remote Arduino Fio Programming
More details coming soon on: http://sites.google.com/site/iz1zckweb/

LICENSE:
EXPEPT FOR ALL PARTS OF CODE I GOT ONLINE where other license restrictions may apply
(that's a lot! See Credits below) this project is released under 
The MIT License (MIT) http://opensource.org/licenses/MIT
The above copyright notice  shall be included in all copies or substantial portions of this Software.

LIMITATION OF RESPONSIBILITY:
This project is distributed as-is in the hope that it may be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
or FITNESS FOR A PARTICULAR PURPOSE. You are free to adapt the code the to
your needs but I can't be held responsible for any phisical or electronic 
damage, electric shock or injuries. You should ALWAYS DOUBLE CHECK all connections
and make sure you TAKE ALL THE SAFETY MEASURES related to projects where electric
currents are involved.

CREDITS:
This project is built copy-pasting and adjusting lots of code preciously shared 
by the arduino & hacking community around the world, and specifically...

- BMP085 Pressure sensor
    http://bildr.org/2011/06/bmp085-arduino/
    http://www.sparkfun.com/tutorials/253
- RTC Clock
    http://learn.adafruit.com/ds1307-real-time-clock-breakout-board-kit/understanding-the-code
- SD Card
    http://arduino.cc/en/Tutorial/Datalogger
- Arduino Fio Programming
    http://arduino.cc/en/Main/ArduinoBoardFioProgramming
- Temperature DS18S20 sensor
    http://bildr.org/2011/07/ds18b20-arduino/
- DHT22 Humidity sensor
    http://gist.github.com/dmccreary/7503212
- LED as Light sensor
    http://www.instructables.com/id/LED-as-lightsensor-on-the-arduino/
- Vcc internal sensor
    http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
    http://www.semifluid.com/2012/09/09/arduino-fio-internal-voltmeter-and-thermometer/
- Wind sensor / Vane / Rain gauge
    http://kesslerarduino.wordpress.com/2012/06/21/sparkfun-weather-station/
- Barometric Pressure sensor
    http://bildr.org/2011/06/bmp085-arduino/
- Arduino Sleep Mode
    http://playground.arduino.cc/Learning/arduinoSleepCode
- Arduino Fio and XBee Sleep Modes
    http://www.semifluid.com/2012/09/07/arduino-fio-low-power-setup/
- XBee
    http://www.johnhenryshammer.com/WOW2/pagesHowTo/xbeeSeries1.php
    http://www.desert-home.com/p/the-world-of-xbee.html
- Raspberry Pi and ThingSpeak (Python script)
    http://www.desert-home.com/2013/09/raspberry-pi-and-thingspeak.html
    http://pyserial.sourceforge.net/
