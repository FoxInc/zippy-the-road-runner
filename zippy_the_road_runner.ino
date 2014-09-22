
#include <Motor.h>
#include "QTRSensors_teensy3.h"

#define BLUETOOTH Serial

#define ON_LINE(sensor)((sensor<COMPARE))
#define OVERSHOOT_LINE_TIME 50

// PID Constants
#define KP	2
#define KI	0
#define KD	80

// MAX SPEED
int SPEED_MAX = 2048;
#define SPEED_CALIBRATE 786
#define SPEED_TURN 1024

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

unsigned char flag = 0;

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
	case 'C':
		LED_LEFT_ON;
		//LED_RIGHT_OFF;
		motorLeft.write(-speed);
		motorRight.write(speed);
		delay(delayTime);
		LED_LEFT_OFF;
		LED_RIGHT_OFF;
		break;

	case 'D':
	case 'R':
		LED_RIGHT_ON;
		//LED_LEFT_OFF;
		motorLeft.write(speed);
		motorRight.write(-speed);
		delay(delayTime);
		LED_LEFT_OFF;
		LED_RIGHT_OFF;
		break;

	case 'S':
		motorLeft.write(speed);
		motorRight.write(speed);
		delay(delayTime);
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
	for (unsigned int i = 0; i < 80; i++)
	{
		if (i == 0 || i == 55)
			turn('L', SPEED_CALIBRATE, 20);
		if (i == 20)
			turn('R', SPEED_CALIBRATE, 20);

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
	runMappingMode();
}

void runMappingMode()
{
	while (1)
	{
		followSegment();

		

		unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;
		turn('S', 512, 10);
		position_ = readSensors();

		if (FOUND_LEFT())
			foundLeft = 1;
		if (FOUND_RIGHT())
			foundRight = 1;

		if (foundLeft && foundRight)
		{
			turn('S', 512, 60);

			position_ = readSensors();

			if (FOUND_STRAIGHT())
			{
				path[pathCounter++] = 'J';
				path[pathCounter] = '\0';

				LED_LEFT_ON;
				LED_RIGHT_ON;
				delay(20);
				LED_LEFT_OFF;
				LED_RIGHT_OFF;
				simplifyPath();

			}
		}

		char direction = selectTurn(foundLeft, foundRight, foundStraight);
		turn(direction, SPEED_TURN, 200);

		path[pathCounter++] = direction;
		BLUETOOTH.println(path[pathCounter - 1]);
	}
}


void simplifyPath()
{
	sCounter = 0;
	int i;

	// Add the last square's turns as it is
	for (i = 1; i <= 3; i++)
		simplifiedPath[sCounter++] = path[pathCounter - 1 - i];

	for (i = pathCounter - 5; i >= 0; i--)
	{
		switch (path[i])
		{
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

	for (i = 3; i < sCounter; i++)
	{
		if (simplifiedPath[i] == 'L' && simplifiedPath[i + 1] == 'L' && simplifiedPath[i + 2] == 'L' && simplifiedPath[i + 3] == 'L' && simplifiedPath[i + 4] == 'R')
		{
			simplifiedPath[i] = 'R';
			simplifiedPath[i + 1] = 'R';
			simplifiedPath[i + 2] = 'L';
			simplifiedPath[i + 3] = 'O';
			simplifiedPath[i + 4] = 'O';


		}
		if (simplifiedPath[i] == 'R' && simplifiedPath[i + 1] == 'R' && simplifiedPath[i + 2] == 'R' && simplifiedPath[i + 3] == 'R' && simplifiedPath[i + 4] == 'L')
		{
			simplifiedPath[i] = 'L';
			simplifiedPath[i + 1] = 'L';
			simplifiedPath[i + 2] = 'R';
			simplifiedPath[i + 3] = 'O';
			simplifiedPath[i + 4] = 'O';


		}
	}
	sCounter = 0;

	/*while (1)
		{

		BLUETOOTH.println(path);
		BLUETOOTH.println(" :) ");
		BLUETOOTH.println(simplifiedPath);
		}*/

	/*LED_LEFT_ON;
	LED_RIGHT_ON;*/

	while (1)
	{

		unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;
		//SPEED_MAX = 2500;
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
			if (simplifiedPath[sCounter] == 'O')
				sCounter++;
			turn(simplifiedPath[sCounter++], SPEED_TURN, 200);

		}

		if (simplifiedPath[sCounter] == 'J')
		{

			BLUETOOTH.println("END OF GAME !!");
	
			turn('S', 1024, 50);

			DISABLE_STANDBY;

			while (1)
			{
				LED_LEFT_ON;
				LED_RIGHT_ON;
			}
		}
	}
}



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
