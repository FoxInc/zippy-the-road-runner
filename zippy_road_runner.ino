
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
#define SPEED_TURN 1024

// Enable for white line on black background
#define WHITE_ON_BLACK 1
#define COMPARE 300

// QTR 8RC SETUP
#define NUMBER_OF_SENSORS 8
#define EMITTER_ON 1
#define EMITTER_PIN 22
#define TIMEOUT 2500

// Motor Driver Configurations
#define MOTOR_DRIVER_PIN_STANDBY 7
#define ENABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, HIGH)
#define DISABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, LOW)

// Buzzer Settings
#define BUZZER_PIN 23
#define BUZZER_ON digitalWrite(BUZZER_PIN, HIGH);
#define BUZZER_OFF digitalWrite(BUZZER_PIN, LOW);

#define DEBUG_MODE 0

motor motorLeft, motorRight;

// Sensor configs
unsigned char sensorPins[] = { 14, 15, 16, 17, 18, 19, 20, 21 };
unsigned short sensorValues[NUMBER_OF_SENSORS];

// PID Variables
float position_ = 0, proportional = 0, derivative = 0, integral = 0, lastProportional = 0;
float control = 0;

// Record the path
char path[100];
unsigned int pathCounter = 0;

// Initialize QTR8RC Sensor Array with sensor pins array
// and number of sensors as second parameter
QTRSensorsRC qtrRC(sensorPins, NUMBER_OF_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned char FOUND_LEFT()
{
	return (ON_LINE(sensorValues[0]) && ON_LINE(sensorValues[1]) && ON_LINE(sensorValues[2]) && ON_LINE(sensorValues[3]) && ON_LINE(sensorValues[4]));
}

unsigned char FOUND_RIGHT()
{
	return (ON_LINE(sensorValues[7]) && ON_LINE(sensorValues[6]) && ON_LINE(sensorValues[5]) && ON_LINE(sensorValues[4]) && ON_LINE(sensorValues[3]));
}

unsigned char FOUND_STRAIGHT()
{
	return((ON_LINE(sensorValues[2]) || ON_LINE(sensorValues[3])) && (ON_LINE(sensorValues[4]) && ON_LINE(sensorValues[5])));
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
		motorLeft.write(-speed);
		motorRight.write(speed);
		delay(delayTime);
		break;

	case 'R':
		motorLeft.write(speed);
		motorRight.write(-speed);
		delay(delayTime);
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
	pinMode(13, OUTPUT);

	//DISABLE_STANDBY;
	ENABLE_STANDBY;

	pinMode(BUZZER_PIN, OUTPUT);

	motorLeft.setPins(6, 5, 4);
	motorRight.setPins(8, 9, 10);

	motorLeft.setMaxSpeed(SPEED_MAX);
	motorRight.setMaxSpeed(SPEED_MAX);

	motorLeft.initialise();
	motorRight.initialise();

	//12 bit width of analog write values
	analogWriteResolution(12);

	//BLUETOOTH.begin(9600);

	digitalWrite(13, HIGH);
	BUZZER_ON;
	initializeBot();
	BUZZER_OFF;
	digitalWrite(13, LOW);

}

void initializeBot()
{
	// Calibrate sensors 
	for (unsigned int i = 0; i < 75; i++)
	{
		if (i == 0)
			turn('L', SPEED_CALIBRATE, 0);
		if (i == 20)
			turn('R', SPEED_CALIBRATE, 0);
		if (i == 55)
			turn('L', SPEED_CALIBRATE, 0);

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
	followSegment();
	followSegment();
	followSegment();
	runMappingMode();
}

void followSegment()
{
	position_ = readSensors();

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

void runMappingMode()
{

	unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;
	// C - Left 90 Junction
	// D - Right 90 Junction
	// T - Point Junction
	// L - Left 90
	// R - Right 90

	//followSegment();

	position_ = readSensors();

	// Starting of intersection
	if (FOUND_LEFT())
		foundLeft = 1;
	if (FOUND_RIGHT())
		foundRight = 1;

	if (foundLeft || foundRight)
	{
		/*BLUETOOTH.print("foundLeft");
		BLUETOOTH.println(foundLeft);

		BLUETOOTH.print("foundRight");
		BLUETOOTH.println(foundRight);
		*/

		//motorLeft.setSpeed(-1024);
		//motorRight.setSpeed(-1024);

		//motorLeft.stop();
		//motorRight.stop();

		//delay(30);

		//position_ = readSensors();

		if (foundLeft && foundRight)
		{
			//delay(10);
			while (FOUND_LEFT() || FOUND_RIGHT())
				position_ = readSensors();

			if (FOUND_STRAIGHT())
			{

				path[pathCounter++] = 'J';
				path[pathCounter] = '\0';
				motorLeft.write(-1024);
				motorRight.write(-1024);
				delay(50);
				DISABLE_STANDBY;
				BLUETOOTH.println(path[pathCounter - 1]);
				BLUETOOTH.println(path);
				while (1)
					BUZZER_ON;
			}
			else
				foundLeft = 1;/*
			if (!FOUND_STRAIGHT)
			{
				BUZZER_ON;
				path[pathCounter++] = 'T';
				BLUETOOTH.println(path[pathCounter - 1]);
				foundLeft = 1;
			}*/

			BUZZER_OFF;
		}

		else if (foundLeft && !foundRight)
		{
			BUZZER_ON;
			while (FOUND_LEFT())
			{
				position_ = readSensors();
			}
			if (FOUND_STRAIGHT())
			{
				BUZZER_ON;
				path[pathCounter++] = 'C';
				BLUETOOTH.println(path[pathCounter - 1]);
				foundLeft = 1;
			}
			else
			{
				path[pathCounter++] = 'L';
				BLUETOOTH.println(path[pathCounter - 1]);
				foundLeft = 1;
			}
			BUZZER_OFF;

		}

		else if (foundRight && !foundLeft)
		{
			BUZZER_ON;
			while (FOUND_RIGHT())
			{
				position_ = readSensors();
			}
			if (!FOUND_STRAIGHT())
			{

				path[pathCounter++] = 'R';
				BLUETOOTH.println(path[pathCounter - 1]);
				foundRight = 1;
			}

			else
			{
				BUZZER_ON;
				path[pathCounter++] = 'D';
				BLUETOOTH.println(path[pathCounter - 1]);
				foundRight = 1;

			}
			BUZZER_OFF;
		}


		//BUZZER_ON;
		////delay(30);
		//motorLeft.write(-32);
		//motorRight.write(-32);
		//

		//delay(70);
		//BUZZER_OFF;

		//position_ = readSensors();

		//// appx in the middle of intersection
		///*if (ON_LINE(sensorValues[0]))
		//	foundLeft = 1;
		//	if (ON_LINE(sensorValues[7]))
		//	foundRight = 1;

		//	delay(OVERSHOOT_LINE_TIME / 2);*/

		////Ahead of intersection
		///*if (ON_LINE(sensorValues[0]))
		//	foundLeft = 1;
		//if (ON_LINE(sensorValues[7]))
		//	foundRight = 1;*/

		//if ( (ON_LINE(sensorValues[2]) || ON_LINE(sensorValues[3]) ) && ( ON_LINE(sensorValues[4]) && ON_LINE(sensorValues[5]) ))
		//	foundStraight = 1;

		////'+' Junction Stop the Motors
		//if (foundLeft && foundRight && foundStraight)
		//{
		//	path[pathCounter++] = 'J';
		//	path[pathCounter] = '\0';
		//	motorLeft.stop();
		//	motorRight.stop();
		//	DISABLE_STANDBY;
		//	BLUETOOTH.println(path[pathCounter - 1]);
		//	BLUETOOTH.println(path);
		//	while (1)
		//		BUZZER_ON;
		//}

		char direction = selectTurn(foundLeft, foundRight, foundStraight);
		turn(direction, SPEED_TURN, 200);

		/*motorLeft.stop();
		motorRight.stop();

		while (1);*/

		// left 90 Junction
		//if (foundStraight)
		//{
		//	// Left 90 Junction
		//	if (foundLeft && !foundRight)
		//	// T point Junction
		//	{
		//		path[pathCounter++] = 'C';
		//		BLUETOOTH.println(path[pathCounter - 1]);
		//	}
		//	// Right 90 Junction
		//	else if (foundRight && !foundLeft)
		//	{

		//		path[pathCounter++] = 'D';
		//		BLUETOOTH.println(path[pathCounter - 1]);
		//	}
		//}
		//else if (foundLeft && foundRight && !foundStraight)
		//{

		//	path[pathCounter++] = 'T';
		//	BLUETOOTH.println(path[pathCounter - 1]);
		//}

		//else if (foundLeft || foundRight)
		//{
		//	path[pathCounter++] = direction;
		//	BLUETOOTH.println(path[pathCounter - 1]);
		//}

	}
}