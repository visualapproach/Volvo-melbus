# Volvo MELBUS

## Bluetooth and custom text in the display

### This is my latest code and schematics. If anyone misses the old files, tell me and I'll put them up again.
But I think this is so much better than earlier versions so I deleted them.


Parts list:
* Arduino Nano 5V
* Transistor that can handle more than 0,5A
* Resistors:
   * 3x 100Ω
   * 1x 1KΩ
* A rectifier diode (> 0,5A e.g. 1N4004)
* 1x 470 uF capacitor
* Cables and optionally some connectors for fast removal of device from the car. (not included in schematics)
* Male DIN plug, 8 pin, 270 degrees
* CSR8645 Bluetooth device with 12v regulator onboard
* 4066 bilateral switch IC (optional to remote control audio source)
* Ground loop isolator (Optional. Use if you hear noise)

Some knowledge of electronics, and you need to be good at soldering. You are responsible if things break! Not me! 

Credits to 
* [Karl Hagström](http://gizmosnack.blogspot.se/2015/11/aux-in-volvo-hu-xxxx-radio.html)
* http://volvo.wot.lv/wiki/doku.php?id=melbus.
* https://www.vincentgijsen.nl/

Thanks to archi for making a PCB

This project shows the way I did it. I'm a hobbyist, not a professional electronics engineer. Proceed at your own risk. The devices use different voltage levels, so there is a chance that you will let the magic smoke out. (Hence the 4066)

Demo https://youtu.be/jGOKGUjlLhI

## Schematics
(R5 and R6 are optional. They are there to measure battery voltage. The zener diode is not needed but protects the arduino if R5 breaks)
![schematics](schematics/bat_mon.png)

