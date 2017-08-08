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
![schematics](/schematics_melbus_hack_v2.png)

A picture of my circuit. I cut the audio GND cable after I took the picture, and put everything in a 3d-printed case.
![Image](/IMG_2056.png)

Just because I could, I added a couple of RGB LEDs to the foot dwell light armatures. It is controlled by pressing the CD buttons on the HU. I can turn on or off driver's/passengers side individually by pressing CD 1 / 2. And CD3, 4 and 5 toggles the RED, GREEN and BLUE LEDs (Possible colors is red, green, blue, cyan, magenta, yellow and white). I drilled a 5 mm hole just below the lightbulb connector and pressfitted the LED. I think it turned out pretty nice!
![Image](/melbus_LEDs.jpg)

If anybody knows anything about the communication between HU and SAT radio, I'd appreciate some help!
