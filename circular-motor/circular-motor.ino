/* Linear (forward-only) control of a DC motor
 *
 *
 * Pinout:
 *
 * P9 -> PWM to Motor (Darlington switch, ie)
 * P2 -> Optical encoder input A
 * P3 -> Optical encoder input B
 */

#define DEBUG_VERBOSE	0

#include <Encoder.h>

const int pinPWM = 9;
const int pinEncoderA = 2;
const int pinEncoderB = 3;

const int pwmMinimum = 90;
const int pwmMaximum = 255;

Encoder encMotor(2, 3);

void setup() {
	pinMode(pinEncoderA, INPUT);
	pinMode(pinEncoderB, INPUT);
	Serial.begin(9600);

	/* Get a direction */
	Serial.print("Steps: ");
}

int dir = -1;
int neg = 0;
long posMotorPast = 0;
long posMotorFuture = 0;
int rateMotor = 0;

void readNextPosition() {
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (dir >= 0) {
				/* Go there */
				dir *= (neg ? -1 : 1);
				Serial.print("\r\nPWM: "); Serial.print(dir); Serial.print("\r\n");
				posMotorFuture += dir;
			}

			/* Get a direction */
			Serial.print("\r\nSteps: ");

			dir = -1;
			neg = 0;
		} else if (c == '-' && neg == 0 && dir < 0) {
				neg = 1;
			Serial.write(c);
		} else if (c >= '0' && c <= '9') {
			if (dir < 0)
				dir = 0;
			dir *= 10;
			dir += (c - '0');
			Serial.write(c);
		} else if (c == '\b') {
			Serial.print("\b \b\b");
		} else {
			Serial.write('\a');
			Serial.print("BEEP[");
			Serial.print(c);
			Serial.print("] dir=");
			Serial.print(dir);
			Serial.print(", neg=");
			Serial.print(neg);
			Serial.print("\r\n");
		}
	}
}

void loop() {
	long posMotorNow = encMotor.read();

	if (posMotorNow >= posMotorFuture) {
		/* We are where we want to be */
		rateMotor = 0;
		analogWrite(pinPWM, 0);
		posMotorNow = encMotor.read();
		if (posMotorNow > posMotorFuture) {
			Serial.print("Overstepped: ");
			Serial.println(posMotorNow - posMotorFuture);
			posMotorFuture = posMotorNow;
		}
		readNextPosition();
	} else {
		int speed;

		speed = posMotorFuture - posMotorNow;
		if (speed > pwmMaximum)
			speed = pwmMaximum;
		if (speed < pwmMinimum)
			speed = pwmMinimum;

		/* Moving ahead... */
		analogWrite(pinPWM, speed);
#if DEBUG_VERBOSE
		Serial.print("MC: speed=");
		Serial.print(speed);
		Serial.print(", now=");
		Serial.print(posMotorNow);
		Serial.print(", future=");
		Serial.println(posMotorFuture);
#endif
	}
}


