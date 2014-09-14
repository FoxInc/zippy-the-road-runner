
#include <Motor.h>
#include "QTRSensors_teensy3.h"

// PID Constants
#define KP	10
#define KI	0
#define KD	80

// MAX SPEED
#define SPEED_MAX 4095
#define SPEED_CALIBRATE 768

// Enable for white line on black background
#define WHITE_ON_BLACK 0

// QTR 8RC SETUP
#define NUMBER_OF_SENSORS 8
#define EMITTER_ON 1
#define EMITTER_PIN 22
#define TIMEOUT 2500

#define BUZZER_PIN 23

// Motor Driver Configurations
#define MOTOR_DRIVER_PIN_STANDBY 7
#define ENABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, HIGH)
#define DISABLE_STANDBY digitalWrite(MOTOR_DRIVER_PIN_STANDBY, LOW)

// Buzzer Settings
#define BUZZER_ON digitalWrite(BUZZER_PIN, HIGH);
#define BUZZER_OFF digitalWrite(BUZZER_PIN, LOW);

#define DEBUG_MODE 1

motor motorLeft, motorRight;

// Sensor configs
unsigned char sensorPins[] = { 14, 15, 16, 17, 18, 19, 20, 21 };
unsigned short sensorValues[NUMBER_OF_SENSORS];

//unsigned char ROBOT_STATE = 0;
//String bluetoothBuffer;
//char bluetoothChar;

// PID Variables
float position_ = 0, proportional = 0, derivative = 0, integral = 0, lastProportional = 0;
float control = 0;

// Record the path
String path;
unsigned int pathCounter = 0;

// Initialize QTR8RC Sensor Array with sensor pins array
// and number of sensors as second parameter
QTRSensorsRC qtrRC(sensorPins, NUMBER_OF_SENSORS, TIMEOUT, EMITTER_PIN);

void turn(char direction)
{
	switch (direction)
	{
	case 'L':
		motorLeft.write(-SPEED_CALIBRATE);
		motorRight.write(SPEED_CALIBRATE);
		break;

	case 'R':
		motorLeft.write(SPEED_CALIBRATE);
		motorRight.write(-SPEED_CALIBRATE);
		break;

	case 'S':
		motorLeft.stop();
		motorRight.stop();
		break;
	}

}

void initializeBot()
{
	// Calibrate sensors 
	for (unsigned int i = 0; i < 90; i++)
	{
		if (i == 0)
			turn('L');
		if (i == 30)
			turn('R');
		if (i == 70)
			turn('L');
		
		// Emitters on
		if (WHITE_ON_BLACK == 1 && EMITTER_ON == 1)
			qtrRC.calibrate(QTR_EMITTERS_ON);

		// Emitters off
		else
			qtrRC.calibrate();

	}
	//Display values serially

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

void setup()
{
	if (DEBUG_MODE == 1)
	{
		Serial.begin(115200);
	}

	//BLUETOOTH.begin(9600);

	pinMode(MOTOR_DRIVER_PIN_STANDBY, OUTPUT);
	pinMode(13, OUTPUT);

	ENABLE_STANDBY;

	pinMode(BUZZER_PIN, OUTPUT);

	motorLeft.setPins(5, 6, 4);
	motorRight.setPins(9, 8, 10);

	motorLeft.setMaxSpeed(SPEED_MAX);
	motorRight.setMaxSpeed(SPEED_MAX);

	motorLeft.initialise();
	motorRight.initialise();

	//12 bit width of analog write values
	analogWriteResolution(12);

	digitalWrite(13, HIGH);
	BUZZER_ON;
	initializeBot();
	BUZZER_OFF;
	digitalWrite(13, LOW);

}

void runMappingMode()
{
	// All sensors on white
	//while (sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
	//{
	//	BUZZER_ON;
	//	position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);

	//}
	//if (position_ == 0 || position_ == 7000)
	//	turn('L');
	//else if (position_ > 2500 && position_ < 4500)
	//{
	//	motorLeft.stop();
	//	motorRight.stop();
	//}



	// 11110000 or 11100000 left 90degree turn
	//if (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750 && (sensorValues[3] > 750 || sensorValues[4] > 750))
	//{
	//	if (oneInchLine == 'B')
	//	{
	//		turn('L');
	//		// B - left 
	//		path[pathCounter++] = 'L';
	//	}
	//	//White Line Ahead
	//	else if (oneInchLine == 'F')
	//	{
	//		turn('L');
	//		// B - Left
	//		path[pathCounter++] = 'B';

	//	}
	//}

	//// 00001111 or 00000111 right 90degree turn
	//else if (sensorValues[6] > 750 && sensorValues[7] > 750 && sensorValues[2] > 750 && (sensorValues[5] > 750 || sensorValues[4] > 750))
	//{
	//	//Nothing Ahead
	//	if (oneInchLine == 'B')
	//	{
	//		turn('R');
	//		path[pathCounter++] = 'J';
	//	}
	//	//White Line Ahead
	//	else if (oneInchLine == 'F')
	//	{
	//		// A - Right
	//		turn('A');
	//		//Reached the end of MAPPPING_MODE
	//	}
	//}

}


void loop()
{

	//delay(500);
	if (WHITE_ON_BLACK == 1)
		position_ = qtrRC.readLine(sensorValues, QTR_EMITTERS_ON, true);

	else
		position_ = qtrRC.readLine(sensorValues);

	//runMappingMode();		

	proportional = position_ - 3500;

	derivative = proportional - lastProportional;
	lastProportional = proportional;
	integral += proportional;

	control = proportional *KP + integral*KI + derivative*KD;

	if (DEBUG_MODE == 1)
	{
		Serial.print("Control : ");
		Serial.print(control);
		Serial.print("\t");
		Serial.print("Position : ");
		Serial.print(proportional);
		Serial.print("\t");
		Serial.print("Values : ");
		showSensorValues();
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
