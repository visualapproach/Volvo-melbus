# Volvo MELBUS

Bluetooth enable your Volvo stereo HU.

What you need:
* Arduino Nano 5V
* Transistor that can handle more than 0,5A
* Resistors:
   * 3x 100Ω
   * 3x 560Ω
   * 1x 1KΩ
* A rectifier diode (> 0,5A e.g. 1N4004)
* 3x LED's (To drop voltage and look cool)
* 1x 470 uF capacitor
* Cables
* Male DIN plug, 8 pin, 270 degrees
* [BT Audio Module like this](http://www.ebay.com/itm/161854077325?_trksid=p2057872.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT "Ebay Link")

Some knowledge of electronics. You are responsible if things break! Not me!

Based on other peoples work. Credits to [Karl Hagström](http://gizmosnack.blogspot.se/2015/11/aux-in-volvo-hu-xxxx-radio.html) and http://volvo.wot.lv/wiki/doku.php?id=melbus.

This project shows the way I did it. I'm a hobbyist, not a professional electronics engineer. Proceed at your own risk. I nearly blew the BT module and the Arduino by switching the 12v RUN cable with the 5v BUSY cable...

## Schematics

![Image](/IMG_2340.JPG)
![Image](/IMG_2341.JPG)
![Image](/IMG_2342.JPG)

