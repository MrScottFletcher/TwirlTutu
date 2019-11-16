// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       TwirlTutu.ino
    Created:	11/10/2019 8:50:38 AM
    Author:     DESKTOP-62IO2EV\scott
*/

/* ==================================================================================================================================================
		 Project: NeoPixel Playground
Neopixel chipset: ws2812B  (144 LED/m strip)
		  Author: Scott C
		 Created: 12th June 2015
	 Arduino IDE: 1.6.4
		 Website: http://arduinobasics.blogspot.com/p/arduino-basics-projects-page.html
	 Description: This project will allow you to cycle through and control five LED
				  animation sequences using a potentiometer and an accelerometer
					 Sequence 1:   Cylon with Hue Control                                       Control: Potentiometer only
					 Sequence 2:   Cylon with Brightness Control                                Control: Potentiometer only
					 Sequence 3:   Comet effect with Hue and direction control                  Control: Potentiometer and Accelerometer (Y axis only)
					 Sequence 4:   FireStarter / Rainbow effect with Hue and Direction control  Control: Potentiometer and Accelerometer (Y axis only)
					 Sequence 5:   Digital Spirit Level                                         Control: Accelerometer only (Y axis)

				  This project makes use of the FastLED library. Some of the code below was adapted from the FastLED library examples (eg. Cylon routine).
				  The Comet, FireStarter and Digital Spirit Level sequence was designed by ScottC.
				  The FastLED library can be found here: http://fastled.io/
				  You may need to modify the code below to accomodate your specific LED strip. See the FastLED library site for more details.
===================================================================================================================================================== */

#include <Adafruit_LSM303_U.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <math.h> 
#include <Adafruit_NeoPixel.h>

//This project needs the FastLED library - link in the description.
#include "FastLED.h"

//The total number of LEDs being used is 144
//#define NUM_LEDS 144
//#define NUM_LEDS 24
#define NUM_LEDS 74

// The data pin for the NeoPixel strip is connected to digital Pin 6 on the Arduino
#define DATA_PIN 6

// The internal data pin for the single NeoPixel is connected to digital Pin 8 on the Flora
#define NEOPIXELPIN 8


//Initialise the LED array, the LED Hue (ledh) array, and the LED Brightness (ledb) array.
CRGB leds[NUM_LEDS];
byte ledh[NUM_LEDS];
byte ledb[NUM_LEDS];

//Pin connections
const int potPin = A0;      // The potentiometer signal pin is connected to Arduino's Analog Pin 0
const int yPin = A4;        // Y pin on accelerometer is connected to Arduino's Analog Pin 4
							// The accelerometer's X Pin and the Z Pin were not used in this sketch

//Global Variables ---------------------------------------------------------------------------------
byte g_hueVal;                // g_hueVal:      stores the potentiometer signal value
byte preHueVal = 0;          // preHueVal:  stores the previous potentiometer value
int LEDSpeed = 1;             // LEDSpeed:    stores the "speed" of the LED animation sequence
int maxLEDSpeed = 50;       // maxLEDSpeed: identifies the maximum speed of the LED animation sequence
int LEDAccel = 0;             // LEDAccel:    stores the acceleration value of the LED animation sequence (to speed it up or slow it down)
int LEDPosition = 30;         // LEDPosition: identifies the LED within the strip to modify (leading LED). The number will be between 0-143.  (Zero to NUM_LEDS-1)
int oldPos = 0;               // oldPos:      holds the previous position of the leading LED
byte g_hue = 0;               // g_hue:         stores the leading LED's g_hue value
byte g_streamIntensity = 255;       // g_streamIntensity:   the default brightness of the leading LED
byte bright = 80;           // bright:      this variable is used to modify the brightness of the trailing LEDs
int animationDelay = 0;     // animationDelay: is used in the animation Speed calculation. The greater the animationDelay, the slower the LED sequence.
int effect = 0;             // effect:      is used to differentiate and select one out of the four effects
int sparkTest = 0;          // sparkTest:   variable used in the "sparkle" LED animation sequence 
boolean constSpeed = false; // constSpeed:  toggle between constant and variable speed.

int g_proposedOriginLED = 30;
uint8_t rainbowBrightness = 120;
CRGBPalette16 currentPalette;
TBlendType    currentBlending;

int g_energyFactor = 50;

int16_t accelleration_strength = 0;
int16_t accelleration_vector = 0;
int16_t accelleration_vectorRotationOffset = 0;
int accelleration_ModeSwitchThreshold = 100000;
const int ACCELL_SAMPLE_COUNT = 50;
//const int ACCELL_UPDATE_EVERY = 100;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Adafruit_NeoPixel dot = Adafruit_NeoPixel(1, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);


//hard coded for now
static int originLED = 10;

//===================================================================================================================================================
// setup()
//===================================================================================================================================================
void setup() {
  //=============================================
  //LEAVE THIS HERE FOR BOOTUP TESTING!!!
	delay(2000);          //Delay for two seconds to power the LEDS before starting the data signal on the Arduino
	dot.begin();
	dot.setBrightness(50);
	dot.show(); // Initialize all pixels to 'off'

	colorWipeDot(dot.Color(255, 0, 0), 500); // Red
	colorWipeDot(dot.Color(0, 255, 0), 500); // Green
	colorWipeDot(dot.Color(0, 0, 255), 500); // Blue
	rainbowCycleDot(2);
//=============================================

	FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);                            //initialise the LED strip       
	LEDPosition = originLED;

	colorWipeDot(dot.Color(255, 0, 0), 500); // Red
	Serial.begin(9600);
	colorWipeDot(dot.Color(0, 255, 0), 500); // Green
	// Try to initialise and warn if we couldn't detect the chip
	if (!accel.begin())
	{
		Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
		while (1);
	}
	colorWipeDot(dot.Color(0, 0, 255), 500); // Blue
	//dot.setBrightness(0);
	//dot.show(); // Initialize all pixels to 'off'
}


//===================================================================================================================================================
// loop() : The Arduino will take readings from the potentiometer and accelerometer to control the LED strip
//===================================================================================================================================================
void loop() {
	adjustSpeed();
	//constrainLEDs();
	LoopLEDs();
	computeVector();
}

void computeVector() {

	UpdateMotionStats();
	SetOriginAndMode(g_proposedOriginLED, g_energyFactor);
}

void UpdateMotionStats() {

	static int16_t sampleLoopNumber = 0;

	sensors_event_t event;
	accel.getEvent(&event);

	static float accelHistory_ForwardBack[ACCELL_SAMPLE_COUNT];
	static float accelHistory_UpDown[ACCELL_SAMPLE_COUNT];
	static float accelHistory_LeftRight[ACCELL_SAMPLE_COUNT];

	dot.setBrightness(25);
	switch (sampleLoopNumber % 3)
	{
	case 0:
		colorWipeDot(dot.Color(255, 0, 0), 1); // Red
		break;
	case 1:
		colorWipeDot(dot.Color(0, 255, 0), 1); // Green
		break;
	case 2:
		colorWipeDot(dot.Color(0, 0, 255), 1); // Blue
		break;
	}

	float this_ForwardBack = 0;
	float this_UpDown = 0;
	float this_LeftRight = 0;

	//Due to the orientation of the sensor pack...
	accelHistory_ForwardBack[sampleLoopNumber] = event.acceleration.y;
	accelHistory_UpDown[sampleLoopNumber] = event.acceleration.x;
	accelHistory_LeftRight[sampleLoopNumber] = event.acceleration.z;

	this_ForwardBack = accelHistory_ForwardBack[sampleLoopNumber];
	this_UpDown = accelHistory_UpDown[sampleLoopNumber];
	this_LeftRight = accelHistory_LeftRight[sampleLoopNumber];

	float avg_ForwardBack = 0;
	float avg_UpDown = 0;
	float avg_LeftRight = 0;

	for (size_t i = 0; i < ACCELL_SAMPLE_COUNT; i++)
	{
		if (i != sampleLoopNumber) {
			avg_ForwardBack += accelHistory_ForwardBack[i];
			avg_UpDown += accelHistory_UpDown[i];
			avg_LeftRight += accelHistory_LeftRight[i];
		}
	}

	//don't count the current one
	avg_ForwardBack = avg_ForwardBack / (ACCELL_SAMPLE_COUNT - 1);
	avg_UpDown = avg_UpDown / (ACCELL_SAMPLE_COUNT - 1);
	avg_LeftRight = avg_LeftRight / (ACCELL_SAMPLE_COUNT - 1);

	bool bChanged = false;

	float energy = 0;
	if (this_ForwardBack != 0 && abs(this_ForwardBack) > abs(avg_ForwardBack))
	{
		energy += abs(this_ForwardBack);
		//colorWipeDot(dot.Color(255, 255, 0), 500); // Yellow
		bChanged = true;
	}
	if (this_UpDown != 0 && abs(this_UpDown) > abs(avg_UpDown))
	{
		energy += abs(this_UpDown);
		bChanged = true;
	}

	if (this_LeftRight != 0 && abs(this_LeftRight) > abs(avg_LeftRight))
	{
		energy += abs(this_LeftRight);
	}

	if (bChanged == true)
	{
		//kick it
		float rad = atan2(this_LeftRight * 100, avg_ForwardBack * 100);
		float deg = rad * (180 / PI);
		float absDeg = abs(deg);
		float absLed = absDeg / 2.25;

		g_proposedOriginLED = (int)absLed;

		g_energyFactor = ((energy - 10) / 10.0) * 100;
		if (g_energyFactor > 100)
			g_energyFactor = 100;
		if (g_energyFactor < 1)
			g_energyFactor = 0;

	}

	//Be sure to loop the counter around to zero again...
	sampleLoopNumber = (sampleLoopNumber + 1) % ACCELL_SAMPLE_COUNT;

}

//=================================================
void SetOriginAndMode(int g_proposedOriginLED, int g_energyFactor) {

	static int idleLoopCount = 0;

	originLED = g_proposedOriginLED;

	if (g_energyFactor > 90) {
		//and sparkle!
		idleLoopCount = 0;
		rainbowSpin(1000, 100, 200);
	}
	if (g_energyFactor < 20) {
		idleLoopCount++;
		//jump to it
		//and sparkle!
		if (idleLoopCount > 100) 
		{
			idleLoopCount = 0;

			rainbowSpin(10, 40, 200);
		}
	}
	else {
		idleLoopCount = 0;
		mirrorStreamWithHueControl();

		//Here's some other unused stuff to manage the direction, etc.
		//int ledDiff = 0;
		//int energySteps = 0;
		//bool diffIsForward = false;
		//if (g_proposedOriginLED > originLED) {
		//	ledDiff = (g_proposedOriginLED - originLED);
		//	if (ledDiff > NUM_LEDS / 2) {
		//		//leaving this here in case we want to do other backwards stuff.
		//		diffIsForward = false;
		//	}
		//	else {
		//		diffIsForward = true;
		//	}
		//}
		//if (g_proposedOriginLED < originLED) {
		//	ledDiff = (originLED - g_proposedOriginLED);
		//	if (ledDiff > NUM_LEDS / 2) {
		//		//go forwards
		//		diffIsForward = true;
		//	}
		//	else {
		//		//leaving this here in case we want to do other backwards stuff.
		//		diffIsForward = false;
		//	}
		//}
		//now split the diff based on energy
	}
}
//=================================================

void rainbowSpin(uint8_t speedValue, int breakoutEnergyThreshold, int maxLoopCount)
{
	currentPalette = RainbowStripeColors_p;
	currentBlending = LINEARBLEND;
	FastLED.setBrightness(120);

	int iLoopCounter = 0;
	while (g_energyFactor < breakoutEnergyThreshold && iLoopCounter < maxLoopCount)
	{

		UpdateMotionStats();

		static uint8_t startIndex = 0;

		startIndex = startIndex - 1; /* motion speed */

		FillLEDsFromPaletteColors(startIndex);
		FastLED.show();
		FastLED.delay(1000 / speedValue);
		iLoopCounter++;
	}
}

void FillLEDsFromPaletteColors(uint8_t colorIndex)
{

	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = ColorFromPalette(currentPalette, colorIndex, rainbowBrightness, currentBlending);
		colorIndex += 3;
	}
}


//===================================================================================================================================================
// adjustSpeed() : use the Y axis value of the accelerometer to adjust the speed and the direction of the LED animation sequence
//===================================================================================================================================================
void adjustSpeed() {
	// Take a reading from the Y Pin of the accelerometer and adjust the value so that 
	// positive numbers move in one direction, and negative numbers move in the opposite diraction. 
	// We use the map function to convert the accelerometer readings, and the constrain function to ensure that it stays within the desired limits
	// The values of 230 and 640 were determined by trial and error and are specific to my accelerometer. You will need to adjust these numbers to suit your module.

	//LEDAccel = constrain(map(analogRead(yPin), 230, 640, maxLEDSpeed, -maxLEDSpeed), -maxLEDSpeed, maxLEDSpeed);
	//LEDAccel = constrain(map(analogRead(lsm.accelData.x), 230, 640, maxLEDSpeed, -maxLEDSpeed), -maxLEDSpeed, maxLEDSpeed);
	LEDAccel = constrain(map(g_energyFactor, 5, 100, maxLEDSpeed, -maxLEDSpeed), -maxLEDSpeed, maxLEDSpeed);
	//LEDAccel = 0;

	//start at blue, add the energy factor, avoid going past 255
	//g_hueVal = (0 + (g_energyFactor / 100 * 255)) % 255;
	g_hueVal = (g_energyFactor % 255);


	// If the constSpeed variable is "true", then make sure that the speed of the animation is constant by modifying the LEDSpeed and LEDAccel variables.
	if (constSpeed) {
		LEDAccel = 0;
		if (LEDSpeed > 0) {
			LEDSpeed = maxLEDSpeed / 1.1;     // Adjust the LEDSpeed to half the maximum speed in the positive direction
		}
		if (LEDSpeed < 0) {
			LEDSpeed = -maxLEDSpeed / 1.1;    // Adjust the LEDSpeed to half the maximum speed in the negative direction
		}
	}

	// The Speed of the LED animation sequence can increase (accelerate), decrease (decelerate) or stay the same (constant speed)
	LEDSpeed = LEDSpeed + LEDAccel;

	//The following lines of code are used to control the direction of the LED animation sequence, and limit the speed of that animation. 
	if (LEDSpeed > 0) {
		LEDPosition++;                                       // Illuminate the LED in the Next position
		if (LEDSpeed > maxLEDSpeed) {
			LEDSpeed = maxLEDSpeed;                              // Ensure that the speed does not go beyond the maximum speed in the positive direction
		}
	}

	if (LEDSpeed < 0) {
		LEDPosition--;                                       // Illuminate the LED in the Prior position
		if (LEDSpeed < -maxLEDSpeed) {
			LEDSpeed = -maxLEDSpeed;                           // Ensure that the speed does not go beyond the maximum speed in the negative direction
		}
	}
}


//====================================================
// FOR ONBOARD NEOPIXEL
//====================================================
// Fill the dots one after the other with a color
void colorWipeDot(uint32_t c, uint8_t wait) {
	for (uint16_t i = 0; i < dot.numPixels(); i++) {
		dot.setPixelColor(i, c);
		dot.show();
		delay(wait);
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycleDot(uint8_t wait) {
	uint16_t i, j;

	for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
		for (i = 0; i < dot.numPixels(); i++) {
			dot.setPixelColor(i, WheelDot(((i * 256 / dot.numPixels()) + j) & 255));
		}
		dot.show();
		delay(wait);
	}
}
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t WheelDot(byte WheelPos) {
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85) {
		return dot.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	else if (WheelPos < 170) {
		WheelPos -= 85;
		return dot.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	else {
		WheelPos -= 170;
		return dot.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
	}
}


//===================================================================================================================================================
// constrainLEDs() : This ensures that the LED animation sequence remains within the boundaries of the various arrays (and the LED strip)
//                   and it also creates a "bouncing" effect at both ends of the LED strip.
//===================================================================================================================================================
void constrainLEDs() {
	LEDPosition = constrain(LEDPosition, 0, NUM_LEDS - 1);    // Make sure that the LEDs stay within the boundaries of the LED strip
	if (LEDPosition == 0 || LEDPosition == NUM_LEDS - 1) {
		LEDSpeed = (LEDSpeed * -0.9);                         // Reverse the direction of movement when LED gets to end of strip. This creates a bouncing ball effect.
	}
}

void LoopLEDs() {
	LEDPosition = LEDPosition % (NUM_LEDS);
}


//===================================================================================================================================================
// mirrorStreamWithHueControl() :  
//===================================================================================================================================================
void mirrorStreamWithHueControl() {
	constSpeed = false;                                  // Make the LED animation speed constant

	//split and mirror off of origin
	//origin is the 'startng' LED in a 365 degree ring

	if (LEDPosition > NUM_LEDS)
		LEDPosition = 0;
	
	int diff = 0;
	if (LEDPosition < originLED) {
		//maintain our 'positive or negative' orientation to origin when we loop around
		diff = (NUM_LEDS - originLED + LEDPosition) * -1;
	}
	else {
		diff = (originLED - LEDPosition);
	}

	//int diff = (originLED + NUM_LEDS - LEDPosition) % (NUM_LEDS);
	//offest of origin of current LED
	float percentFromEnd = 1;
	int mirrorLED = originLED;
	int ledCountEquiv = abs(diff * 2);
	if (ledCountEquiv >= NUM_LEDS) {
		LEDPosition = originLED;
		mirrorLED = originLED;
		percentFromEnd = 100;
		diff = 0;
	}
	else {
		mirrorLED = (originLED + diff + NUM_LEDS) % (NUM_LEDS);

		//as the leds approach the tail, dim them out
		if (diff < 0) diff = diff * -1;
		if (diff != 0)
			percentFromEnd = float((NUM_LEDS / 2) - diff) / float(NUM_LEDS / 2);
	}

	int new_intensity = g_streamIntensity;
	if (percentFromEnd >= 0) {
		new_intensity = g_streamIntensity * percentFromEnd;
		if (new_intensity < 0) new_intensity = 0;
	}

	showLED(LEDPosition, g_hueVal, 255, new_intensity);    // Illuminate the LED
	showLED(mirrorLED, g_hueVal, 255, new_intensity);      // Illuminate the LED

	fadeLEDs(12);                                       // Fade LEDs by a value of 8. Higher numbers will create a shorter tail.
	setDelay(LEDSpeed);                                 // The LEDSpeed is constant, so the delay is constant
}



//===================================================================================================================================================
// fadeLEDs(): This function is used to fade the LEDs back to black (OFF) 
//===================================================================================================================================================
void fadeLEDs(int fadeVal) {
	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i].fadeToBlackBy(fadeVal);
	}
}


//===================================================================================================================================================
// showLED() : is used to illuminate the LEDs 
//===================================================================================================================================================
void showLED(int pos, byte LEDhue, byte LEDsat, byte LEDbright) {
	leds[pos] = CHSV(LEDhue, LEDsat, LEDbright);
	FastLED.show();
}


//===================================================================================================================================================
// setDelay() : is where the speed of the LED animation sequence is controlled. The speed of the animation is controlled by the LEDSpeed variable.
//              and cannot go faster than the maxLEDSpeed variable.
//===================================================================================================================================================
void setDelay(int LSpeed) {
	animationDelay = maxLEDSpeed - abs(LSpeed);
	delay(animationDelay);
}


//===================================================================================================================================================
// sparkle() : is used by the fireStarter routine to create a sparkling/fire-like effect
//             Each LED g_hue and brightness is monitored and modified using arrays  (ledh[]  and ledb[])
//===================================================================================================================================================
void sparkle(byte hDiff) {
	for (int i = 0; i < NUM_LEDS; i++) {
		ledh[i] = ledh[i] + hDiff;                // hDiff controls the extent to which the g_hue changes along the trailing LEDs

		// This will prevent "negative" brightness.
		if (ledb[i] < 3) {
			ledb[i] = 0;
		}

		// The probability of "re-igniting" an LED will decrease as you move along the tail
		// Once the brightness reaches zero, it cannot be re-ignited unless the leading LED passes over it again.
		if (ledb[i] > 0) {
			ledb[i] = ledb[i] - 2;
			sparkTest = random(0, bright);
			if (sparkTest > (bright - (ledb[i] / 1.1))) {
				ledb[i] = bright;
			}
			else {
				ledb[i] = ledb[i] / 2;
			}
		}
		leds[i] = CHSV(ledh[i], 255, ledb[i]);
	}
}
