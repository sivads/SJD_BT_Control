SJD_BT_Control
==============

Bluetooth Control for ArcBotics Sparki Robot

Programming by S. Davis and J. Davis
Date: 06/2014
```
 ___           _      ___     _         _   _       
|SJ \ __ ___ _(c)___ | _ \___| |__  ___| |_(_)__ ___
| |) / _` \ V / (_-< |   / _ \ '_ \/ _ \  _| / _(_-<
|___/\__,_|\_/|_/__/ |_|_\___/_.__/\___/\__|_\__/__/
```
Copyright (c) 2014, SJ Davis Robotics (Steven E. Davis and Jonathan Davis)

This program aims to demonstrate the full functionality of the ArcBotics Sparki robot
platform using the Bluetooth module to control Sparki from a Bluetooth capable device.
A companion program, written with the MIT App Inventor 2 programming environment, was
developed to use an Android device as the controller (SparkiBTController.aia source).
This controller is available in the Google Play Store as "Sparki Bluetooth Controller"
and must be run on a 7" Tablet such as the Nexus 7.

Commands are sent from the controller to Sparki and Sparki reacts and/or responds with
data read from his various sensors.  The commands are simple one character commands
followed by an optional numeric command value.  For example, the command "F" makes
Sparki move forward and the command "f,10" makes Sparki move forward 10 steps.  The list 
of commands that Sparki responds to can be found in the switch statement in the code
below.  There are also commands for Sparki to read a sensor continuously and report the 
data read back to the controller.  Since the data transmitted back to the controller 
needs to be read and processed before more data is transmitted (asynchronously), the 
controller will send an ACK (command "a") back to Sparki when the data has been processed 
and the controller is ready to receive data again.  If multiple sensors are being read 
and multiple data reports are being transmitted to the controller, a round-robin approach 
is used to send the data from the various sensors in a sequential and controlled manner.           
                
This application was created as a father/son project to fuel my son's education and love 
of technology & robots!  We worked very hard and had lots of fun doing this project
together!  If you like what we did or find this code useful for your own project, please 
consider donating to our education fund so that can continue to do projects like this and 
pursue higher educational goals (hopefully, Jon will go to MIT!)

E-mail/PayPal Donations: donations@sjdavisrobotics.com

BitCoin Donations: 1Hp3htmCAyiYNg52XXhoVtjB7VdkqaQDMq

**Thank you!**

Steve (dad) and Jonathan (7yo son) Davis
* 
Special thanks to the MIT App Inventor team!  I have been a professional electrical 
engineer and programmer for over 30 years and the App Inventor platform is such an 
excellent place for young and beginning programmers to learn and build applications that
are useful and creative!  Thanks so much!! 
*
