# blindController
Jeenode / Arduino Roller Blind Controller - reports via RF to emonhub (Openenergymonitor), remote control via RF, intelligent opening /closing based on light, sun dipping and more.

blindController.pl / blindController.cfg - this is a configuration program to allow you set various paramaters on the blinds (light open/close thresholds, timers, overrun parameters, operation mode and more).  Uses a mysql database to hold the config for all your blinds and send to them at will. blindController.sql holds the SQL needed to create the database.

Also has the ability to read data from a WH1080 weather station and relay it in Jeelib RF packet format.
Finally, a DS18B20 temperature sensor can be connected across the 'OPEN' switch and the code will relay temperature back to the base station too.

Enjoy.

alandpearson / july 2022
