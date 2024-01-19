This repository is to document and organize my files, designs, code for my at home robot fighting league I want to do with my kids. The goal is to mimic the 150g weight robot competitions with cheap semi standarized parts.

I will be focusing on the ESP32-C3 Super Mini as the main micro controller as it is small cheap comes with bluetooth and a library for connecting to Xbox One controllers which I already have on hand.

I will use what seem to be fairly standard brushed motors for the weight size, the N20 DC Brushed motor with various geared ratios. 

I am using MX1508 DC Motor Driver Modules for speed controllers, they are cheap and can easily handle the current requirements of the N20 motors.

I currently have uploaded the code and design for a generic wedge bot, currently it weighs too much (225g), I'm reworking the PCB to be smaller along with new frame designs that are smaller and ligher, the code is a slightly modified version of the bluetooth controller library I am using (Bluepad32) the repository can be found here:
https://github.com/ricardoquesada/bluepad32
This code needs to be cleaned up by removing undeeded code and adding a few features but currently will allow you to steer and drive the robot with variable speed control.

I am using the Arduino IDE since I'm familiar with it.

Most of the parts I'm using come from Aliexpress.
