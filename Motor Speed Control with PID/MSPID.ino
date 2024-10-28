#include <LiquidCrystal.h>
#include "PID.h"

/*
Developerd by: Dr Bharat Verma
Affiliation: ECE Department, The LNMIIT jaipur
LCD Initialisation
#include "PID.h"
	For more detail Please follow the Arduino exmaple
	This project will required L293D driver with encoder motor. Connections are,
	1. Motor input-Output at 3 and 4 respectively
	2. Encoder pin 1 and pin 2 connected to the Arduino pin 2 and 3.
	3. Motor is controlled through IN1 and IN2 for clock wise and anti-clock wise rotations.
	4. IN1 and IN2 is connected to the Arduino PIN 9 and 10.
	5. Setpoint is received with an potentiometer connected at Anaogue Input A0.
	updateLCD() and updateSerial() are used to update the LCD and Serial Monitor respectively.
*/

LiquidCrystal lcd(12, 11, 7, 6, 5, 4);
double R, S, out_n, t1, t2, T, uc, error;
const double kc = 0.1;
const double tauI = 0.03;
const double tauD = 0.004;
const double SaturationMax = 255;
const double SaturationMin = -255;
const double PPR = 280;

/*
	PID opbject from the PID header file
	The given PID controller supports the anti-windup for better performance
*/

PID pid = PID(kc, tauI, tauD, SaturationMax, SaturationMin);

/*Arduino Setup and Loop*/

void setup()
{
	pinMode(3, INPUT);
	pinMode(2, INPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	lcd.begin(16, 4);	// Using 16*4 LCD
	analogWrite(10, 0); // Start Motor
	Serial.begin(9600);
}

void loop()
{
	t1 = pulseIn(3, HIGH);
	t2 = pulseIn(3, LOW);

	T = (t1 + t2);
	S = (1000000 * 60) / T / PPR;

	R = 100; // analogRead(A0) / 2;

	uc = pid.PIDval(R, S);
	if (uc < 0)
	{
		analogWrite(9, -uc);
		analogWrite(10, 0);
	}
	else
	{
		analogWrite(9, 0);
		analogWrite(10, uc);
	}
	error = pid.error;

	// updateLCD();
	updateSerial();
}

void updateLCD()
{
	lcd.setCursor(0, 0);
	lcd.print("Set Point=");
	lcd.print(R);
	lcd.setCursor(0, 1);
	lcd.print("RPM=");
	lcd.print((int)S);
	lcd.setCursor(0, 2);
	lcd.print("Error=");
	lcd.print((float)error);
	lcd.setCursor(0, 3);
	lcd.print("u=");
	lcd.print(uc);
}

void updateSerial()
{
	Serial.print("--- \n OUTPUT = ");
	Serial.println(S);
	Serial.print("INPUT = ");
	Serial.println(uc);
}
