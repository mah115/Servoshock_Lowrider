/*
 This example reads the Dualshock 4 controller state from the Servoshock controller and outputs it to the serial console.

Pressing the right-side joystick button will put make the Servoshock take control of the digital and servo outputs.
Pressing it again will return control to the PS4 controller.

 It will also set the controller LEDs to a changing rainbow color and the controller will rumble when the triggers are pulled.

 Set the serial baud rate to 115200 bps.


The basic structure of the program is:

	#include <SPI.h>
	#include "servoshock_PS4.h"

	// set the slave select pin for the Servoshock.  Set jumper JP3 on the Shield to D10 if using digital output 10.
	const int slaveSelect = 10;
	Servoshock Servoshock1(slaveSelect);  //create instance of Servoshock

	void setup()
	{
		//initialize SPI:
		digitalWrite(slaveSelect, HIGH);
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV16);
		SPI.setBitOrder(MSBFIRST);
		SPI.begin();
	}

	void loop()
	{
		Servoshock1.Update(); //this updates the input and output SPI packets.

		//Rest of the program, for example:
		if (Servoshock1.inPacket.dPadUp)
		{
			...
		}

		//if you want to control a servo:
		Servoshock1.outPacket.overrideLStickX = 1;
		Servoshock1.outPacket.lStickX_uS = servo_uS; //output this pulse width in microseconds.

		delay(10); //don't update faster than 100Hz.
	}
 */


// include the SPI library:
#include <SPI.h>
#include <stdint.h>
#include "servoshock_PS4.h"
#include "print_serial.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//add new program to list here, can do a fine/replace to rename
typedef enum {
	STANDBY = 0,
	PROG_1,
	PROG_2,
	PROG_3,
} PROGRAM_STATE;

// set the slave select pin for the Servoshock.  Set jumper JP2 on the Shield to D10 if using digital output 10.
const uint16_t slaveSelect = 10;
Servoshock Servoshock1(slaveSelect);  //create instance of Servoshock

//last button states we need to track to detect state change
bool tpadPressLast = 0;


PROGRAM_STATE currentProgram = STANDBY; //this indicates which program to run.  default STANDBY
uint32_t programTimer = 0;  //this keeps track of how many loops since the start of a program override we've gone though
unsigned int LEDCounter = 0;//counter for manipulating the LED color
unsigned char LEDRed = 255;
unsigned char LEDGreen = 0;
unsigned char LEDBlue = 0;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);



void RelinquishControl(void)
{
	Servoshock1.outPacket.overrideDPadUp = 0;
	Servoshock1.outPacket.overrideDPadRight = 0;
	Servoshock1.outPacket.overrideDPadDown = 0;
	Servoshock1.outPacket.overrideDPadLeft = 0;
	Servoshock1.outPacket.overrideTriangle = 0;
	Servoshock1.outPacket.overrideCircle = 0;
	Servoshock1.outPacket.overrideCross = 0;
	Servoshock1.outPacket.overrideSquare = 0;
	Servoshock1.outPacket.overrideLBumper = 0;
	Servoshock1.outPacket.overrideRBumper = 0;
	Servoshock1.outPacket.overrideLTriggerDigital = 0;
	Servoshock1.outPacket.overrideRTriggerDigital = 0;
	Servoshock1.outPacket.overrideLStickPress = 0;
	Servoshock1.outPacket.overrideRStickPress = 0;
	Servoshock1.outPacket.overrideShare = 0;
	Servoshock1.outPacket.overrideOptions = 0;
	Servoshock1.outPacket.overrideTpadPress = 0;
	Servoshock1.outPacket.overridePsButton = 0;

	Servoshock1.outPacket.overrideLStickX = 0;
	Servoshock1.outPacket.overrideLStickY = 0;
	Servoshock1.outPacket.overrideRStickX = 0;
	Servoshock1.outPacket.overrideRStickY = 0;
	Servoshock1.outPacket.overrideLTriggerAnalog = 0;
	Servoshock1.outPacket.overrideRTriggerAnalog = 0;
	Servoshock1.outPacket.overrideLTpadX = 0;
	Servoshock1.outPacket.overrideLTpadY = 0;
	Servoshock1.outPacket.overrideRTpadX = 0;
	Servoshock1.outPacket.overrideRTpadY = 0;
	Servoshock1.outPacket.overrideTiltX = 0;
	Servoshock1.outPacket.overrideTiltY = 0;

	Servoshock1.outPacket.overrideLED = 0;
	Servoshock1.outPacket.overrideRumbleH = 0;
	Servoshock1.outPacket.overrideRumbleL = 0;
}


/***Interpolate the servo PWM pulse width, given the two keyframes.  Return 0 if out of bounds.
 *  inputs:
 * 		*channel: reference to servoshock servo PWM output register
 * 		currentTime: current value of program timer counter
 * 		time1: servo move start time
 * 		pulseWidth1: servo pulse width at start time
 * 		time2: servo move end time
 * 		pulseWidth2: servo pulse width at end time
 ***/
int ServoInterpolate( uint16_t *channel, uint32_t currentTime, uint32_t time1, uint16_t pulseWidth1, uint32_t time2, uint16_t pulseWidth2 ) {
	if (currentTime < time1 || currentTime > time2 || time1 > time2)
	{
		return 0; //return 0 if time is out of bounds
	}
	//if not out of bounds, interpolate and write channel PWM register
	*channel = (pulseWidth1*(time2-currentTime) + pulseWidth2*(currentTime-time1))/(time2-time1);
	return 1; //success
}

/***Interpolate the Adafruit PCA 9685 PWM board pulse width, given the two keyframes.  Return 0 if out of bounds.
 *  inputs:
 * 		pwm_object: Adafruit_PWMServoDriver object
 * 		channel: channel number, 0-15
 * 		currentTime: current value of program timer counter
 * 		time1: PWM  start time
 * 		pulseWidth1: pulse width at start time, 0-4095
 * 		time2: PWM end time
 * 		pulseWidth2: pulse width at end time, 0-4095
 ***/
int PwmInterpolate(Adafruit_PWMServoDriver *pwm_object, uint8_t channel, uint32_t currentTime, uint32_t time1, uint16_t pulseWidth1, uint32_t time2, uint16_t pulseWidth2 ) {
	if (currentTime < time1 || currentTime > time2 || time1 > time2)
	{
		return 0; //return 0 if time is out of bounds
	}
	//if not out of bounds, interpolate and write channel PWM register
	pwm_object->setPin(channel, (pulseWidth1*(time2-currentTime) + pulseWidth2*(currentTime-time1))/(time2-time1) );
	return 1; //success
}


void setup() {
	//initialize SPI:
  	SPI.begin();
	digitalWrite(slaveSelect, HIGH);
	SPI.setDataMode(SPI_MODE1);
	SPI.setClockDivider(SPI_CLOCK_DIV128);
	SPI.setBitOrder(MSBFIRST);

	Serial.begin(115200);  //initialize serial monitor

	pwm.begin(); //Initialize PWM board
	pwm.setOscillatorFrequency(27000000);
	pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
	Wire.setClock(100000);

}
void loop() {
	
	//record variables we need to use to detect button state changes
	tpadPressLast = Servoshock1.inPacket.tpadPress;

	Servoshock1.Update(); //send/receive data between servoshock

	////////////////////////THESE OVERRIDES ARE ALWAYS ON////////////////////////
	//Set rumble
	Servoshock1.outPacket.overrideLED = 1;
	Servoshock1.outPacket.overrideRumbleH = 1;
	Servoshock1.outPacket.overrideRumbleL = 1;
	Servoshock1.outPacket.rumbleH = Servoshock1.inPacket.rTriggerAnalog;
	Servoshock1.outPacket.rumbleL = Servoshock1.inPacket.lTriggerAnalog;

	//Set LED
	Servoshock1.outPacket.overrideLED = 1;
	Servoshock1.SetLED(LEDRed, LEDGreen, LEDBlue, 0, 0);
	if (LEDCounter < 255) {
		LEDRed--;
		LEDGreen++;
	} else if (LEDCounter < 510) {
		LEDGreen--;
		LEDBlue++;
	} else if (LEDCounter < 765) {
		LEDBlue--;
		LEDRed++;
	}
	if (LEDCounter < 768) {
		LEDCounter++;
	} else {
		LEDCounter = 0;
	}

	////////////////////////ARDUINO PROGRAMS////////////////////////
	switch (currentProgram) {
		case STANDBY:
			for (int i=0;i<16;i++){
				pwm.setPin(i,0); //ramp up LEDs
			}
			//conditions that will change the state
			if (tpadPressLast == 0 && Servoshock1.inPacket.tpadPress == 1)
			{
				Serial.print("PROG1\n\r");
				currentProgram = PROG_1;
				programTimer = 0;

			}

			break;

		case PROG_1:
			const uint32_t PROG_1_TIME_SCALE = 5;
			Serial.println(programTimer,DEC);
			//conditions that will change the state, e.g. button press or timer
			if ((tpadPressLast == 0 && Servoshock1.inPacket.tpadPress == 1) ||  //if the tpad is pressed again...
				 programTimer == 40*PROG_1_TIME_SCALE) 	//or if the timer expires...
			{
				RelinquishControl();
				programTimer = 0;
				currentProgram = STANDBY; //exit program
				Serial.print("STANDBY\n\r");
				break;
			}

			Servoshock1.outPacket.overrideLStickX = 1;
			Servoshock1.outPacket.overrideLStickY = 1;
			Servoshock1.outPacket.overrideRStickX = 1;
			Servoshock1.outPacket.overrideRStickY = 1;
			ServoInterpolate(&Servoshock1.outPacket.lStickX_uS, programTimer, 0*PROG_1_TIME_SCALE, 1500, 10*PROG_1_TIME_SCALE, 590);  //need to pass by reference (use the '&' before the variable)
			ServoInterpolate(&Servoshock1.outPacket.lStickY_uS, programTimer, 5*PROG_1_TIME_SCALE, 800, 15*PROG_1_TIME_SCALE, 1800);
			ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 17*PROG_1_TIME_SCALE, 1700, 19*PROG_1_TIME_SCALE, 1300);
		    ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 20*PROG_1_TIME_SCALE, 1700, 22*PROG_1_TIME_SCALE, 1300);
            ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 23*PROG_1_TIME_SCALE, 1700, 24*PROG_1_TIME_SCALE, 1300);
            ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 24*PROG_1_TIME_SCALE, 1700, 25*PROG_1_TIME_SCALE, 1300); 
			ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 25*PROG_1_TIME_SCALE, 1700, 26*PROG_1_TIME_SCALE, 1300);
			ServoInterpolate(&Servoshock1.outPacket.rStickY_uS, programTimer, 27*PROG_1_TIME_SCALE, 1700, 28*PROG_1_TIME_SCALE, 1300);

			for (int i=0;i<16;i++){
				PwmInterpolate(&pwm, i, programTimer, 0*PROG_1_TIME_SCALE, 0, 10*PROG_1_TIME_SCALE, 4095); //ramp up LEDs
				PwmInterpolate(&pwm, i, programTimer, 10*PROG_1_TIME_SCALE, 4095, 20*PROG_1_TIME_SCALE, 0); //ramp down LEDs
				PwmInterpolate(&pwm, i, programTimer, 20*PROG_1_TIME_SCALE, 0, 30*PROG_1_TIME_SCALE, 4095); //ramp up LEDs
				PwmInterpolate(&pwm, i, programTimer, 30*PROG_1_TIME_SCALE, 4095, 40*PROG_1_TIME_SCALE, 0); //ramp down LEDs

			}


			break;


		default:
			currentProgram = STANDBY;
			break;
	}

	programTimer ++;
	delay(10);
}

