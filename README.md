# blindController
Jeenode / Arduino Roller Blind Controller - reports via RF to emonhub (Openenergymonitor), remote control via RF, intelligent opening /closing based on light, sun dipping and more.

blindController.pl / blindController.cfg - this is a configuration program to allow you set various paramaters on the blinds (light open/close thresholds, timers, overrun parameters, operation mode and more).  Uses a mysql database to hold the config for all your blinds and send to them at will. blindController.sql holds the SQL needed to create the database.

Also has the ability to read data from a WH1080 weather station and relay it in Jeelib RF packet format.
Finally, a DS18B20 temperature sensor can be connected across the 'OPEN' switch and the code will relay temperature back to the base station too.

If using emonhub, here is the config line you will need to receive each blinds status data, and send data to the blind :

[[2]]
    nodename = example_blind
   firmware =V1_4
   hardware = blindControllerv3
    [[[rx]]]
     names = dest,opMode,bsm0,bsm1,tom,ldrval,temp,position0,position1,fwVersion
     datacodes = b,b,h,h,h,h,h,f,f,h
     scales = 1,1,1,1,1,1,0.1,1,1,1
     units = id,opmode,bsm,bsm,tsm,ldr,temp,pos,pos,ver
    [[[tx]]]
     names = cmd,opmode,nodeid,autoOpenMin,autoCloseMin,sunDipSec,sunDipPosition,openThreshold,closeThreshold,sundiptThreshold,autoWaitMin,manWaitMin,opMode,overRun,underRun,crc
     datacodes =b,b,b,b,b,h,b,h,h,h,b,b,b,b,b,H
     units = cmd,opmode,min,min,sec,pct,thresh,thresh,thresh,min,min,opmode,pct,pct,crc



Enjoy.

alandpearson / july 2022
