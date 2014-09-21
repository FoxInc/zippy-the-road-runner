
#include <Motor.h>
#include "QTRSensors_teensy3.h"

#define BLUETOOTH Serial1

#define ON_LINE(sensor)((sensor<COMPARE))
#define OVERSHOOT_LINE_TIME 50

// PID Constants
#define KP	2
#define KI	0
#define KD	80

// MAX SPEED
#define SPEED_MAX 2048
#define SPEED_CALIBRATE 786
#define SPEED_TURN 800

// Enable for white line on black background
#define WHITE_ON_BLACK 1
#define COMPARE 200

// QTR 8RC SETUP
#define NUMBER_OF_SENSORS 8
#define EMITTER_ON 1
#define EMITTER_PIN 25
#define TIMEOUT 2500

// Motor Driver Configurations
#define MOTOR_DRIVER_PIN_STANDBY 7
#define ENABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, HIGH)
#define DISABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, LOW)

// LED Settings
#define LED_LEFT 16
#define LED_RIGHT 15
#define LED_LEFT_ON digitalWrite(LED_LEFT, HIGH)
#define LED_LEFT_OFF digitalWrite(LED_LEFT, LOW)
#define LED_RIGHT_ON digitalWrite(LED_RIGHT, HIGH)
#define LED_RIGHT_OFF digitalWrite(LED_RIGHT, LOW)


#define DEBUG_MODE 0

motor motorLeft, motorRight;

// Sensor configs
unsigned char sensorPins[] = { 33, 32, 31, 30, 29, 28, 27, 26 };
unsigned short sensorValues[NUMBER_OF_SENSORS];

// PID Variables
float position_ = 0, proportional = 0, derivative = 0, integral = 0, lastProportional = 0;
float control = 0;

// Record the path
char path[100], simplifiedPath[100];
unsigned int pathCounter = 0;
unsigned sCounter = 0;

unsigned char MODE = 0;


// Initialize QTR8RC Sensor Array with sensor pins array
// and number of sensors as second parameter
QTRSensorsRC qtrRC(sensorPins, NUMBER_OF_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned char FOUND_LEFT()
{
	return (ON_LINE(sensorValues[0]	) && ON_LINE(sensorValues[1]) && ON_LINE(sensorValues[2]) && ON_LINE(sensorValues[3]) && ON_LINE(sensorValues[4]));
}

unsigned char FOUND_RIGHT()
{
	return (ON_LINE(sensorValues[7]) && ON_LINE(sensorValues[6]) && ON_LINE(sensorValues[5]) && ON_LINE(sensorValues[4]) && ON_LINE(sensorValues[3]));
}

unsigned char FOUND_STRAIGHT()
{
	return((ON_LINE(sensorValues[2]) && ON_LINE(sensorValues[3])) && (ON_LINE(sensorValues[4]) && ON_LINE(sensorValues[5])));
}

char selectTurn(unsigned char found_left, unsigned char found_right, unsigned char found_straight)
{
	if (found_left)
		return 'L';
	else if (found_right)
		return 'R';
	else if (found_straight)
		return 'S';
}

void turn(char direction, unsigned int speed, unsigned short delayTime)
{
	switch (direction)
	{
	case 'L':
		LED_LEFT_ON;
		LED_RIGHT_OFF;
		motorLeft.write(-speed);
		motorRight.write(speed);
		delay(delayTime);
		LED_LEFT_OFF;
		break;

	case 'R':
		LED_RIGHT_ON;
		LED_LEFT_OFF;
		motorLeft.write(speed);
		motorRight.write(-speed);
		delay(delayTime);
		LED_RIGHT_OFF;
		break;

	case 'S':
		break;
	}

}

int readSensors()
{
	int pos;
	if (WHITE_ON_BLACK == 1)
		pos = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);

	else
		pos = qtrRC.readLine(sensorValues);

	return pos;
}

void setup()
{
	if (DEBUG_MODE == 1)
	{
		Serial.begin(9600);
	}

	BLUETOOTH.begin(9600);

	pinMode(MOTOR_DRIVER_PIN_STANDBY, OUTPUT);

	//DISABLE_STANDBY;
	ENABLE_STANDBY;

	pinMode(LED_LEFT, OUTPUT);
	pinMode(LED_RIGHT, OUTPUT);

	motorLeft.setPins(5, 6, 4);
	motorRight.setPins(8, 9, 10);

	motorLeft.setMaxSpeed(SPEED_MAX);
	motorRight.setMaxSpeed(SPEED_MAX);

	motorLeft.initialise();
	motorRight.initialise();

	//12 bit width of analog write values
	analogWriteResolution(12);

	//BLUETOOTH.begin(9600);
	
	LED_LEFT_ON;
	LED_RIGHT_ON;
	initializeBot();
	LED_LEFT_OFF;
	LED_RIGHT_OFF;

}

void initializeBot()
{
	// Calibrate sensors 
	for (unsigned int i = 0; i < 75; i++)
	{
		if (i == 0)
			turn('L', SPEED_CALIBRATE, 20);
		if (i == 20)
			turn('R', SPEED_CALIBRATE, 20);
		if (i == 55)
			turn('L', SPEED_CALIBRATE, 20);

		// Emitters on
		//if (WHITE_ON_BLACK == 1 && EMITTER_ON == 1)
		//	qtrRC.calibrate(QTR_EMITTERS_ON);

		// Emitters off
		//else
		qtrRC.calibrate();

	}

	//Display values serially in debugMode for Debugging
	if (DEBUG_MODE == 1)
	{
		for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
		{
			Serial.print(qtrRC.calibratedMinimumOn[i]);
			Serial.print(' ');
		}
		Serial.println(' ');
		for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
		{
			Serial.print(qtrRC.calibratedMinimumOff[i]);
			Serial.print(' ');
		}
	}
}

void showSensorValues()
{
	for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
	{
		Serial.print(sensorValues[i]);
		Serial.print(' ');
	}

	Serial.println();

}


void loop()
{

	//delay(500);
	
	//followSegment();
	//followSegment();
	// MODE 0 for mapping mode
	if (MODE == 0)
		runMappingMode();

	
	// Mode 1 for simplifying path
	if (MODE == 1)
		simplifyPath();

	// Mode 2 for running on simplified path
	if (MODE == 2)
		runSimplifiedPath();

}

void simplifyPath()
{

	unsigned char i;

	// Add the last square's turns as it is
	for (i = 3; i > 0; i--)
		simplifiedPath[sCounter++] = path[pathCounter - 1 - i];

	MODE = 2;

	// Invert and simpify the remaining path
	for (i = 0; i < pathCounter - 4; i++)
	{
		// Start with left or right 90 intersections

		switch (path[i])
		{
		case 'C':
			if (path[i + 2] == 'L' && path[i + 3] == 'L' && path[i + 4] == 'L')
			{
				simplifiedPath[sCounter++] == 'R';
				i += 4;
			}

			else
				simplifiedPath[sCounter++] = 'D';

		case 'D':
			if (path[i + 2] == 'R' && path[i + 3] == 'R' && path[i + 4] == 'R')
			{
				simplifiedPath[sCounter++] == 'L';
				i += 4;
			}
			else
				simplifiedPath[sCounter++] = 'C';
			break;

		case 'L':
			simplifiedPath[sCounter++] = 'R';
			break;

		case 'R':
			simplifiedPath[sCounter++] = 'L';
			break;
		}

	}

	simplifiedPath[sCounter++] = 'J';
	simplifiedPath[sCounter] = '\0';
	sCounter = 0;
}

void runSimplifiedPath()
{
	unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;

	followSegment();

	position_ = readSensors();

	// Starting of intersection
	if (FOUND_LEFT())
		foundLeft = 1;
	if (FOUND_RIGHT())
		foundRight = 1;

	if (foundLeft || foundRight)
	{
		BLUETOOTH.println(simplifiedPath[sCounter]);
		turn(simplifiedPath[sCounter++], SPEED_TURN, 200);

	}

	else if (simplifiedPath[sCounter] == 'J')
	{
		motorLeft.write(-1024);
		motorRight.write(-1024);
		BLUETOOTH.println("END OF GAME !!");
		delay(50);
		DISABLE_STANDBY;
		while (1)
			LED_LEFT_ON;
		LED_RIGHT_ON;
	}
}

void runMappingMode()
{
	while (1)
	{
		followSegment();

		//delay(20);

		motorLeft.write(512);
		motorRight.write(512);


		unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;

		position_ = readSensors();

		if (FOUND_LEFT())
			foundLeft = 1;
		if (FOUND_RIGHT())
			foundRight = 1;

		delay(60);

		if (FOUND_STRAIGHT())
			foundStraight = 1;

		if (foundLeft && foundRight && foundStraight)
		{
			path[pathCounter++] = 'J';
			path[pathCounter] = '\0';


			//motorLeft.write(-1024);
			//motorRight.write(-1024);
			delay(20);
			//DISABLE_STANDBY;

			LED_LEFT_ON;
			LED_RIGHT_ON;

			BLUETOOTH.println(path);
			BLUETOOTH.println("END OF MAPPING MODE");

				motorLeft.write(512);
				motorRight.write(512);
				//delay(50);
			LED_LEFT_OFF;
			LED_RIGHT_OFF;
			MODE = 1;
			return;
		}

		// T Junction
		if (foundLeft && foundRight && !foundStraight)
		{
			path[pathCounter++] = 'T';
			BLUETOOTH.println(path[pathCounter - 1]);
			foundLeft = 1;
		}

		// L 90 Junction
		if (foundLeft && !foundRight && foundStraight)
		{
			path[pathCounter++] = 'C';
			BLUETOOTH.println(path[pathCounter - 1]);
			foundLeft = 1;
		}

		// R 90 Junction
		if (!foundLeft && foundRight && !foundStraight)
		{
			path[pathCounter++] = 'D';
			BLUETOOTH.println(path[pathCounter - 1]);
			foundRight = 1;
		}


		char direction = selectTurn(foundLeft, foundRight, foundStraight);
		turn(direction, SPEED_TURN, 200);

		// L 90 Turn OR R 90 Turn
		if ((foundLeft || foundRight) && !foundStraight)
		{
			path[pathCounter++] = direction;
			BLUETOOTH.println(path[pathCounter - 1]);
		}


	}
}

//
////unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;
//	//// C - Left 90 Junction
//	//// D - Right 90 Junction
//	//// T - Point Junction
//	//// L - Left 90
//	//// R - Right 90
//
//	////followSegment();
//
//	//position_ = readSensors();
//
//	//// Starting of intersection
//	//if (FOUND_LEFT())
//	//	foundLeft = 1;
//	//if (FOUND_RIGHT())
//	//	foundRight = 1;
//
//	//if (foundLeft || foundRight)
//	//{
//	//	/*BLUETOOTH.print("foundLeft");
//	//	BLUETOOTH.println(foundLeft);
//
//	//	BLUETOOTH.print("foundRight");
//	//	BLUETOOTH.println(foundRight);
//	//	*/
//
//	//	//motorLeft.setSpeed(-512);
//	//	//motorRight.setSpeed(-512);
//
//	//	//delay(30);
//
//	//	position_ = readSensors();
//
//	//	if (foundLeft && foundRight)
//	//	{
//
//	//		LED_LEFT_ON;
//	//		LED_RIGHT_ON;
//
//	//		while (FOUND_LEFT() || FOUND_RIGHT())
//	//			position_ = readSensors();
//
//	//		//delay(30);
//	//		//position_ = readSensors();
//
//	//		if (FOUND_STRAIGHT())
//	//		{
//	//			// Switch to MODE which simplifies path
//	//			MODE = 1;
//
//	//			path[pathCounter++] = 'J';
//	//			path[pathCounter] = '\0';
//
//
//	//			motorLeft.write(-1024);
//	//			motorRight.write(-1024);
//	//			delay(50);
//	//			DISABLE_STANDBY;
//
//	//			BLUETOOTH.println(path);
//	//			BLUETOOTH.println("END OF MAPPING MODE");
//
//	//			delay(1000);
//	//		}
//	//		else
//	//		{
//	//			path[pathCounter++] = 'T';
//	//			BLUETOOTH.println(path[pathCounter - 1]);
//	//			foundLeft = 1;
//	//		}
//
//	//	}
//
//	//	else if (foundLeft && !foundRight)
//	//	{
//	//		while (FOUND_LEFT())
//	//		{
//	//			position_ = readSensors();
//	//		}
//	//		if (FOUND_STRAIGHT())
//	//		{
//	//			path[pathCounter++] = 'C';
//	//			BLUETOOTH.println(path[pathCounter - 1]);
//	//			foundLeft = 1;
//	//		}
//	//		else
//	//		{
//	//			path[pathCounter++] = 'L';
//	//			BLUETOOTH.println(path[pathCounter - 1]);
//	//			foundLeft = 1;
//	//		}
//
//	//	}
//
//	//	else if (foundRight && !foundLeft)
//	//	{
//	//		while (FOUND_RIGHT())
//	//		{
//	//			position_ = readSensors();
//	//		}
//	//		if (!FOUND_STRAIGHT())
//	//		{
//
//	//			path[pathCounter++] = 'R';
//	//			BLUETOOTH.println(path[pathCounter - 1]);
//	//			foundRight = 1;
//	//		}
//
//	//		else
//	//		{
//	//			path[pathCounter++] = 'D';
//	//			BLUETOOTH.println(path[pathCounter - 1]);
//	//			foundRight = 1;
//
//	//		}
//	//	}
//
//
//
//		//LED_ON;
//		//delay(120);
//		////motorLeft.write(-10);
//		////motorRight.write(-10);
//
//
//		////delay(30);
//		//LED_OFF;
//
//		//position_ = readSensors();
//
//		// appx in the middle of intersection
//		/*if (ON_LINE(sensorValues[0]))
//			foundLeft = 1;
//			if (ON_LINE(sensorValues[7]))
//			foundRight = 1;
//
//			delay(OVERSHOOT_LINE_TIME / 2);*/
//
//		//Ahead of intersection
//		/*if (ON_LINE(sensorValues[0]))
//			foundLeft = 1;
//			if (ON_LINE(sensorValues[7]))
//			foundRight = 1;*/
//
//		//if ((ON_LINE(sensorValues[2]) || ON_LINE(sensorValues[3])) && (ON_LINE(sensorValues[4]) || ON_LINE(sensorValues[5])))
//		//foundStraight = 1;
//
//		////'+' Junction Stop the Motors
//		//if (foundLeft && foundRight && foundStraight)
//		//{
//		//	path[pathCounter++] = 'J';
//		//	path[pathCounter] = '\0';
//		//	motorLeft.write(-1024);
//		//	motorRight.write(-1024);
//		//	delay(20);
//		//	DISABLE_STANDBY;
//		//	BLUETOOTH.println(path[pathCounter - 1]);
//		//	BLUETOOTH.println(path);
//		//	while (1)
//		//		LED_ON;
//		//}
//		//if (!foundStraight && foundLeft && foundRight)
//		//{
//		//	path[pathCounter++] = 'T';
//		//	BLUETOOTH.println(path[pathCounter - 1]);
//		//	foundLeft = 1;
//		//}
//
//		/*char direction = selectTurn(foundLeft, foundRight, foundStraight);
//		turn(direction, SPEED_TURN, 235);*/
//
//		///*motorLeft.stop();
//		//motorRight.stop();
//
//		//while (1);*/
//		//
//		////left 90 Junction
//		//if (foundStraight)
//		//{
//		//	// Left 90 Junction
//		//	if (foundLeft && !foundRight)
//		//		// T point Junction
//		//	{
//		//		path[pathCounter++] = 'C';
//		//		BLUETOOTH.println(path[pathCounter - 1]);
//		//	}
//		//	// Right 90 Junction
//		//	if (foundRight && !foundLeft)
//		//	{
//
//		//		path[pathCounter++] = 'D';
//		//		BLUETOOTH.println(path[pathCounter - 1]);
//		//	}
//		//}
//		//
//
//		//else if (foundLeft || foundRight && !foundStraight)
//		//{
//		//	path[pathCounter++] = direction;
//		//	BLUETOOTH.println(path[pathCounter - 1]);
//		//}
//
//	//}

void followSegment()
{
	while (1)
	{
		position_ = readSensors();

		if (FOUND_LEFT() || FOUND_RIGHT())
			break;

		if (DEBUG_MODE == 1)
			showSensorValues();

		proportional = position_ - 3500;

		derivative = proportional - lastProportional;
		lastProportional = proportional;
		integral += proportional;

		control = proportional *KP + integral*KI + derivative*KD;

		if (DEBUG_MODE == 1)
		{
			Serial.print("Control  ");
			Serial.print(control);
			Serial.print(" Position  ");
			Serial.print(proportional);
			Serial.println("  ");
		}


		if (control > SPEED_MAX)
			control = SPEED_MAX;
		if (control < -SPEED_MAX)
			control = -SPEED_MAX;

		if (control < 0)
		{
			motorLeft.write(SPEED_MAX + control);
			motorRight.write(SPEED_MAX);
		}
		else
		{
			motorLeft.write(SPEED_MAX);
			motorRight.write(SPEED_MAX - control);
		}

	}
}
