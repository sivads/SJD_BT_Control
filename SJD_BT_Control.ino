/****************************************************************************************
 Programming by S. Davis and J. Davis
 (c) SJ Davis Robotics
 Date: 06/2014
                        
This program aims to demonstrate the full functionality of the ArcBotics Sparki robot
platform using the Bluetooth module to control Sparki from a Bluetooth capable device.
A companion program, written with the MIT App Inventor 2 programming environment, was
developed to use an Android device as the controller (SparkiBTController.aia source). 

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

## E-mail/PayPal Donations: donations@sjdavisrobotics.com

## BitCoin Donations: 1Hp3htmCAyiYNg52XXhoVtjB7VdkqaQDMq

** Thank you! ** 
Steve (dad) and Jonathan (7yo son) Davis
 
Special thanks to the MIT App Inventor team!  I have been a professional electrical 
engineer and programmer for over 30 years and the App Inventor platform is such an 
excellent place for young and beginning programmers to learn and build applications that
are useful and creative!  Thanks so much!! 

*****************************************************************************************

This Software is provided under the MIT Open Source License Agreement:
 ___           _      ___     _         _   _       
|SJ \ __ ___ _(c)___ | _ \___| |__  ___| |_(_)__ ___
| |) / _` \ V / (_-< |   / _ \ '_ \/ _ \  _| / _(_-<
|___/\__,_|\_/|_/__/ |_|_\___/_.__/\___/\__|_\__/__/
(c) SJ Davis Robotics                                                

Copyright (c) 2014, SJ Davis Robotics (Steven E. Davis and Jonathan Davis)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE. 

			#     #                         #######                  ### 
			#     #   ##   #    # ######    #       #    # #    #    ### 
			#     #  #  #  #    # #         #       #    # ##   #    ### 
			####### #    # #    # #####     #####   #    # # #  #     #  
			#     # ###### #    # #         #       #    # #  # #        
			#     # #    #  #  #  #         #       #    # #   ##    ### 
			#     # #    #   ##   ######    #        ####  #    #    ### 
			
                                        
                                                                    Width Set to 90 ---->
****************************************************************************************/

#include <Sparki.h> 	// include the sparki library
#include "pitches.h" 	// include a list of pitches

// ---------------------------------------------------------------------------------------
// Version

// Code Version {VERSION _MAJOR}.{VERSION_MINOR}.{VERSION_BUILD}
const int VERSION_MAJOR = 1;
const int VERSION_MINOR = 0;
const int VERSION_BUILD = 0;

// ---------------------------------------------------------------------------------------
// Limits

const int LIMIT_STEPS = 50;			// Limit number of steps forward or backward
const int LIMIT_TURNS = 1800;		// Limit the number of degrees in a turn (left/right)

// ---------------------------------------------------------------------------------------
// Calibration

// This factor is added to the number of degrees specified for a turn to ensure accuracy.
const float TURN_CALIBRATION = 0.08;	

// ---------------------------------------------------------------------------------------
// Other Constants
// Coin flip constants (for a random decision)
const int HEADS = 1;
const int TAILS = 0;

// ---------------------------------------------------------------------------------------
// Range Finder Data

// The ultrasonic range finder sends only one piece of data back to the controller as the
// result of a direct request.  This data return has priority over the continuous data
// streams (mag, acc, etc.) -- i.e., range finder does not participate in the round-robin
// data transmission scheme.

int nRangeInCentimeters = 0;
boolean bNewRangeData = false;

// ---------------------------------------------------------------------------------------
// Magnetic Sensor Data

// Flag to indicate that the magnetic sensor should be read and sent to the controller.
boolean bSendingMagData = false;

// Raw Data
float magSensorX = 0.0;
float magSensorY = 0.0;
float magSensorZ = 0.0;

// ---------------------------------------------------------------------------------------
// Accelerometer Sensor Data

// Flag to indicate that the Accelerometer sensor should be read and sent to the 
// controller.
boolean bSendingAccData = false;

// Raw data 
float accSensorX = 0.0;
float accSensorY = 0.0;
float accSensorZ = 0.0;

// ---------------------------------------------------------------------------------------
// Light Sensor Data

// Flag to indicate that the Accelerometer sensor should be read and sent to the 
// controller.
boolean bSendingLgtData = false;

// Raw data 
float lgtSensorL = 0.0;	// Left
float lgtSensorC = 0.0;	// Center
float lgtSensorR = 0.0;	// Right

// ---------------------------------------------------------------------------------------
// Infrared Reflectance Sensor Data

// Flag to indicate that the Accelerometer sensor should be read and sent to the 
// controller.
boolean bSendingIrrData = false;

// Raw data 
float irrSensorEdgeLeft = 0.0;		// Edge Left
float irrSensorLineLeft = 0.0;		// Line Left
float irrSensorLineCenter = 0.0;	// Line Center
float irrSensorLineRight = 0.0;		// Line Right
float irrSensorEdgeRight = 0.0;		// Edge Right

// ---------------------------------------------------------------------------------------
// Data Communication Synchronization and Coordination

// Data pending token:
// Set to true after Sparki sends data to the controller.
// The controller send the "a" command (ACK) to set the data pending flag back to false.
// This flag is used for async flow control to control the flow of messages to the 
// controller (especially necessary for data that is read constantly and sent to the 
// controller). 
boolean bDataPending = false;

// When Sparki is sending data form multiple sensors to the controller, in order to
// prevent blocking, a round-robin data transmission scheme is used.  A roster array is
// used to list the participants in the round-robin scheme.  The nWhoseTurn variable 
// holds the code of the participant whose turn it is currently to send data.  See
// _isMyTurn() and _NextTurn() functions for more details.

int nWhoseTurn = 1;
const int TURN_MAG_DATA = 1;	// Magnetic Data
const int TURN_ACC_DATA = 2;	// Accelerometer Data
const int TURN_LGT_DATA = 3;	// Light Sensor Data
const int TURN_IRR_DATA = 4;	// IR Sensor Data

const int TURN_NUMBER_MAX = 4;

// Round-Robin Roster
const int turnRoster[] = {TURN_MAG_DATA, TURN_ACC_DATA, TURN_LGT_DATA};

// ---------------------------------------------------------------------------------------
// Misc Control Flags

// LCD write flag
boolean bWriteLCD = true;

// ---------------------------------------------------------------------------------------
// Commands

// The single (first character) command read from the controller
char cmdCode = '0';
int cmdValue = 0;


// =========================================================================
// INIT
// =========================================================================
void setup() 
{
	
	// Turn off the LED
	_LEDOff();

	// initialize to look forward
	sparki.servo(SERVO_CENTER);

	// open the gripper
	_openGripper();

	// Set up Bluetooth 
	Serial1.begin(9600);
	delay(200); 

	// Make some noise to indicate initialization complete
	_SparkiPlay_StartUpSound();

} // END INIT


// =========================================================================
// MAIN LOOP
// =========================================================================
void loop()
{
	
_start_:
	
	// RANGE DATA:
	// Give priority to Range Data (that is not being sent continuously)
	if (bNewRangeData) {
		if (!bDataPending) {
		
			// Send Range Data
			_sendRangeData();
			
			// Set data pending flag
			bDataPending = true;
			
			// Reset new range data flag
			bNewRangeData = false;
		}
	}
	else {

		// MAGNETIC SENSOR DATA:
		// Read the magnetic sensor data and send to controller	
		if ((nWhoseTurn == TURN_MAG_DATA) && !bDataPending) {

			if (bSendingMagData) {

				magSensorX = sparki.magX();
				magSensorY = sparki.magY();
				magSensorZ = sparki.magZ();

				Serial1.print("M,");
				Serial1.print(magSensorX);
				Serial1.print(",");
				Serial1.print(magSensorY);
				Serial1.print(",");
				Serial1.println(magSensorZ);

				// Set data pending flag
				bDataPending = true;
			}
			_setNextTurn();
		}
		
		// ACCELEROMETER SENSOR DATA:
		// Read the Accelerometer sensor data and send to the controller	
		if ((nWhoseTurn == TURN_ACC_DATA) && !bDataPending) {
		
			if (bSendingAccData) {

				accSensorX = sparki.accelX();
				accSensorY = sparki.accelY();
				accSensorZ = sparki.accelZ();

				Serial1.print("G,");
				Serial1.print(accSensorX);
				Serial1.print(",");
				Serial1.print(accSensorY);
				Serial1.print(",");
				Serial1.println(accSensorZ);

				// Set data pending flag
				bDataPending = true;
			}
			_setNextTurn();
		}
		
		// LIGHT SENSOR DATA:
		// Read the light sensor data and send to the controller	
		if ((nWhoseTurn == TURN_LGT_DATA) && !bDataPending) {
		
			if (bSendingLgtData) {

				// Read light sensor data from Sparki
				lgtSensorL = sparki.lightLeft();
				lgtSensorC = sparki.lightCenter();
				lgtSensorR = sparki.lightRight();

				// Send light sensor data to controller
				Serial1.print("T,");
				Serial1.print(lgtSensorL);
				Serial1.print(",");
				Serial1.print(lgtSensorC);
				Serial1.print(",");
				Serial1.println(lgtSensorR);

				// Set data pending flag
				bDataPending = true;
			}
			_setNextTurn();
		}

		// INFRARED REFLECTANCE SENSOR DATA:
		// Read the infrared reflectance sensor data and send to the controller	
		if ((nWhoseTurn == TURN_IRR_DATA) && !bDataPending) {
		
			if (bSendingIrrData) {

				// Read infrared reflectance sensor data from Sparki
				irrSensorEdgeLeft = sparki.edgeLeft();
				irrSensorLineLeft = sparki.lineLeft(); 
				irrSensorLineCenter = sparki.lineCenter(); 
				irrSensorLineRight = sparki.lineRight();
				irrSensorEdgeRight = sparki.edgeRight();	

				// Send infrared reflectance sensor data to controller
				Serial1.print("I,");
				Serial1.print(int(irrSensorEdgeLeft));
				Serial1.print(",");
				Serial1.print(int(irrSensorLineLeft));
				Serial1.print(",");
				Serial1.print(int(irrSensorLineCenter));
				Serial1.print(",");
				Serial1.print(int(irrSensorLineRight));
				Serial1.print(",");
				Serial1.println(int(irrSensorEdgeRight));

				// Set data pending flag
				bDataPending = true;
			}
			_setNextTurn();
		}


	} // if (bNewRangeData)
		
	
	// COMMAND HANDLER:	
	// Read the incomming data (if available)
	if (Serial1.available() > 0) {
	
		// Get the single character command
		cmdCode = (char)Serial1.read();
		// Reset the value (optional command suffix)
		cmdValue = 0;
		
		// Do action based on which command was received
		switch(cmdCode) {
			case 'H':
				// Sparki says hello!
				delay(1000);
				_SparkiPlay_HelloSound();
				break;
			case 'F':
				// Move forward (until stop command is received)
				sparki.moveForward();
				break;
		  	case 'f':
		  		// Step Forward with number of steps (second parameter is number of steps)
		  		cmdValue = _readValue();
		  		cmdValue = _limitSteps(cmdValue);
		  		if (cmdValue > 0) {
		  			sparki.moveForward(cmdValue);
		  		}
		  		break;
		  	case 'B':
				// Move backward (until stop command is received)
				sparki.moveBackward();
				break;
		  	case 'b':
		  		// Step Backward with number of steps (second parameter is number of steps)
		  		cmdValue = _readValue();
		  		cmdValue = _limitSteps(cmdValue);
		  		if (cmdValue > 0) {
		  			sparki.moveBackward(cmdValue);
		  		}
		  		break;
		  	case 'L':
				// Move left (until stop command is received)
				sparki.moveLeft();
				break;
		  	case 'l':
		  		// Turn Left with Angle (second parameter is the angle in degrees)
		  		cmdValue = _readValue();
		  		cmdValue = _limitTurns(cmdValue);
		  		cmdValue = _calibrateTurn(cmdValue);
		  		if (cmdValue > 0) {
		  			sparki.moveLeft(cmdValue);
		  		}
		  		break;
		  	case 'R':
				// Move right (until stop command is received)
				sparki.moveRight();
				break;
		  	case 'r':
		  		// Turn Right with Angle (second parameter is the angle in degrees)
		  		cmdValue = _readValue();
		  		cmdValue = _limitTurns(cmdValue);
		  		cmdValue = _calibrateTurn(cmdValue);
		  		if (cmdValue > 0) {
		  			sparki.moveRight(cmdValue);
		  		}
		  		break;
		  	case 'S':
				// Stop 
				sparki.moveStop();
				break;
			case 'O':
				// Open the gripper (until stop gripper is received)
				sparki.gripperOpen();
				break;
			case 'C':
				// Close the gripper (until stop gripper is received)
				sparki.gripperClose();
				break;
			case 'X':
				// Stop gripper
				sparki.gripperStop();
				break;
			case 'M':
				// Starts magnetic data streaming
				bSendingMagData = true;
				break;
			case 'm':
				// Stops magnetic data streaming
				bSendingMagData = false;
				break;
			case 'G':
				// Starts Accelerometer data streaming
				bSendingAccData = true;
				break;
			case 'g':
				// Stops Accelerometer data streaming
				bSendingAccData = false;
				break;
			case 'T':
				// Starts light sensor data streaming
				bSendingLgtData = true;
				break;
			case 't':
				// Stops light sensor data streaming
				bSendingLgtData = false;
				break;
			case 'I':
				// Starts Infrared Reflectance sensor data streaming
				bSendingIrrData = true;
				break;
			case 'i':
				// Stops Infrared Reflectance sensor data streaming
				bSendingIrrData = false;
				break;
			case '1':
				// Range Left - measures the distance to object on left
				nRangeInCentimeters = _getRangeLeft();
				bNewRangeData = true;
				break;
			case '2':
				// Range Left of Center (LC)
				nRangeInCentimeters = _getRangeLC();
				bNewRangeData = true;
				break;
            case '3':
				// Range Center
				nRangeInCentimeters = _getRangeCenter();
				bNewRangeData = true;
				break;
			case '4':
				// Range Right of Center (RC)
				nRangeInCentimeters = _getRangeRC();
				bNewRangeData = true;
				break;
			case '5':
				// Range Right - measures the distance to object on right
				nRangeInCentimeters = _getRangeRight();
				bNewRangeData = true;
				break;
			case 'a':
				// Equivalent to ACK 
				// Sent to Sparki to acknowlege that message was handled by the 
				// controller.  Helps to keep communications in sync.
				bDataPending = false;
				break;
			case 'Q':
				// Beep
				_SparkiBeep(1);
				break;
			case 'q':
				// Shave and a Haircut
				_SparkiPlay_ShaveAndAHaircut();
				break;
			case 'w':
				// LED Off
				_LEDOff();
				break;
			case 'x':
				// LED Red
				_LEDRed();
				break;
			case 'y':
				// LED Green
				_LEDGreen();
				break;
			case 'z':
				// LED Blue
				_LEDBlue();
				break;

		} // Switch
		
		bWriteLCD = true;
	
	}

	// Write to LDC
	if (bWriteLCD) {
		sparki.clearLCD();

		// Program Title and Version
		sparki.print("BT Control v");
		sparki.print(VERSION_MAJOR);
		sparki.print(".");
		sparki.print(VERSION_MINOR);
		sparki.print(".");
		sparki.println(VERSION_BUILD);                
		
		// Program Authors
		sparki.println("___           _");
		sparki.println("SJ \\ __ ___ _(c)___");
		sparki.println(" |) / _` \\ V / (_-<");
		sparki.println("___/\\__,_|\\_/|_/__/");
		sparki.println("(c)SJ Davis Robotics");

		// Command
		sparki.print("Command: ");   
		sparki.println(cmdCode); 
		// Steps
		sparki.print("Value: ");   
		sparki.print(cmdValue); 
		
		// Write to LCD
		sparki.updateLCD();
	
		bWriteLCD = false;
	
	} // Write LCD
	
	
}  // END MAIN LOOP

// =========================================================================
// Helper Functions
// =========================================================================

//--------------------------------------------------------------------
// Data Communication Synchronization and Coordination Helpers

// Next Turn:
// After a member of the round-robin roster completes his turn, he must call this proc 
// to pass control to the next member of the roster.
void _setNextTurn() {
	nWhoseTurn++;
	if (nWhoseTurn > TURN_NUMBER_MAX) {
		nWhoseTurn = 1;
	}
} // END _setNextTurn


//--------------------------------------------------------------------
// Movement Helpers

// Add turn calibration factor to the user specified turn in degrees.
// The calibration factor (TURN_CALIBRATION) was obtained by measuring the undershoot
// angle for a 360-deg turn, then expressing the factor as a multiplier to give the
// additional angle needed to complete a true 360-deg turn.  This factor was tested
// for a 90, 180, and 270-deg turn as well.
int _calibrateTurn(int nDegrees) {
	float fTrueTurn = nDegrees + (TURN_CALIBRATION * nDegrees);
	return int(fTrueTurn);
} // END _calibrateTurn

// Limit the number of steps that can be specified
int _limitSteps(int nSteps) {
	if (nSteps > 0) {
		if (nSteps > LIMIT_STEPS) {
			return LIMIT_STEPS;
		}
		else {
			return nSteps;
		}
	}
	else {
		return 0;
	}
} // END _limitSteps

// Limit the number of degrees for a turn that can be specified.
int _limitTurns(int nTurns) {
	if (nTurns > 0) {
		if (nTurns > LIMIT_TURNS) {
			return LIMIT_TURNS;
		}
		else {
			return nTurns;
		}
	}
	else {
		return 0;
	}
} // END _limitTurns


//--------------------------------------------------------------------
// Command Processing Helpers

// Reads the value part of the command.  Commands sent via BT are in the form of:
// {alpha_character}{numeric_digit1}{numeric_digit1}...{numeric_digitn}{;}
// Basically, a single character command followed by an integer value (one or several 
// digits).  Instead of using something like atoi() to convert a char array to an int 
// value, this proc uses a simple base 10 algorithm that successively multiplies the
// incoming digits by 10 to build the complete value.
int _readValue() {
	
	char cRead = '0';
	int nResult = 0;
	int nDelayCount = 0;
	boolean bContinue = true;
	while ( !Serial1.available() ) {
		delay(10);
		nDelayCount++;
		if (nDelayCount > 500) { break; }
    }
	while ((Serial1.available() > 0) && (bContinue == true)) {
		cRead = (char)Serial1.read();
		if (cRead == ';') { // expected terminator
			bContinue = false;
		}
		else {
			if (('0' <= cRead) && cRead <= '9') { // Must be a numeric digit
				nResult = 10 * nResult + (cRead & 0x0F);
			}
		}
	}
	return nResult;
	
} // END _readValue

//--------------------------------------------------------------------
// Ultrasonic Range Finder Functions

// Get the range in centimeters to Spark'ls LEFT.
int _getRangeLeft() {
	return _getRange(-85);
} // END _getRangeLeft

// Get the range in centimeters to Spark'ls RIGHT.
int _getRangeRight() {
	return _getRange(85);
} // END _getRangeRight

// Get the range in centimeters to Spark'ls CENTER.
int _getRangeCenter() {
	return _getRange(0);
} // END _getRangeCenter

// Get the range in centimeters to Spark'ls LEFT of CENTER (LC).
int _getRangeLC() {
	return _getRange(-40);
} // END _getRangeLC

// Get the range in centimeters to Spark'ls RIGHT of CENTER (RC).
int _getRangeRC() {
	return _getRange(40);
} // END _getRangeRC

// Generic Range Position and Ping
int _getRange(int nPosition) {
	sparki.servo(nPosition);
	delay(200);
	int cm = sparki.ping();
	delay(200);
	sparki.servo(SERVO_CENTER);
	return cm;
} // END _getRange

// Send Range Data to Controller via Bluetooth
void _sendRangeData() {

	// The range data is a global variable:
	// nRangeInCentimeters
	Serial1.print("R,");
	Serial1.println(nRangeInCentimeters);

} // END _sendRangeData

//--------------------------------------------------------------------
// Gripper Control Functions

// Open the Gripper
void _openGripper() {
	sparki.gripperOpen();
	delay(4000);
	sparki.gripperStop();
}  // END _openGripper


// Close the Gripper
void _closeGripper() {
	sparki.gripperClose();
	delay(4000);
	sparki.gripperStop();
}  // END _closeGripper

//--------------------------------------------------------------------
// LED Control Functions

// Turn LED solid Green
void _LEDGreen() {
	sparki.RGB(0, 255, 0); 	// Make the LED maximum Green
} // END _LEDGreen

// Turn LED solid Red
void _LEDRed() {
	sparki.RGB(255, 0, 0); 	// Make the LED maximum Red
} // END _LEDRed

// Turn LED solid Blue
void _LEDBlue() {
	sparki.RGB(0, 0, 255); 	// Make the LED maximum Blue
} // END _LEDBlue

// Turn LED OFF
void _LEDOff() {
	sparki.RGB(RGB_OFF);
} // END _LEDOff

// Turn LED solid white (all colors on)
void _LEDWhite() {
	sparki.RGB(255, 255, 255);
} // END _LEDWhite

//--------------------------------------------------------------------
// Utility Functions

// Coin Flip:
// Returns Heads or Tails according to defined constants.
int _flipCoin() {
	int nCoin = random(0,2);
	if (nCoin == 0) {
		return TAILS;
	}
	else {
		return HEADS;
	}
} // END _flipCoin

//--------------------------------------------------------------------
// Sparki's Sounds

const int MORSE_DOT = 16;
const int MORSE_DASH = 6;

void _SparkiBeep(int n) {
	if ( n > 0 ) {
		do {
			sparki.beep();
			delay(500);
			n = n - 1;
		} while(n > 0);
	}
} // END _SparkiBeep


// Play Startup Sound (init complete)
void _SparkiPlay_StartUpSound() {

	// notes in the melody:
	// Morse Code R for Right:  .-.
	int melody[] = { NOTE_C5, NOTE_A5};

	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 4 };

	_SparkiPlayMelody(melody, noteDurations, 2);

}  // END _SparkiPlay_StartUpSound

// Play Hello Sound
void _SparkiPlay_HelloSound() {

	// notes in the melody:
	int melody[] = { NOTE_A5, NOTE_C5};

	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 4 };

	_SparkiPlayMelody(melody, noteDurations, 2);

}  // END _SparkiPlay_HelloSound

// Play Shave and a Haircut Melody
void _SparkiPlay_ShaveAndAHaircut() {
	// notes in the melody:

	int melody[] = { NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4 };
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
	_SparkiPlayMelody(melody, noteDurations, 8);
} // END _SparkiPlay_ShaveAndAHaircut

// Melody Player:
// Plays the specified melody (notes and durations)
void _SparkiPlayMelody(int melody[], int durations[], int nSize) {

	// NOTE:
	// Rats.  C arrays don't store their own sizes anywhere, so sizeof() only 
	// works the way you expect if the size is known at compile time. 
	// malloc() is treated by the compiler as any other function, so sizeof() 
	// can't tell that melody is an array, let alone how big it is. 
	// If you need to know the size of the array, you need to explicitly pass 
	// it to your function, either as a separate argument, or by using a struct 
	// containing a pointer to your array and its size.

	for (int thisNote = 0; thisNote < nSize; thisNote++) {

		// calculate the note duration as 1 second divided by note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000/durations[thisNote];

		sparki.beep(melody[thisNote],noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;

		delay(pauseBetweenNotes);

		// stop the tone playing:
		sparki.noBeep();
	}
} // END _SparkiPlayMelody

