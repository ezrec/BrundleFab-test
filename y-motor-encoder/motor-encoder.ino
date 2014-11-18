/* Linear (forward-only) control of a DC motor
 *
 *
 * Pinout:
 *
 * M1 -> DC Motor control (Adafruit Motor Shield v1)
 * P18 -> Optical encoder input A
 * P14 -> Optical encoder input B
 */

#define DEBUG_VERBOSE	0

#include <Wire.h>
#include <AFMotor.h>
#include <Encoder.h>

#define AXIS_OVERSHOOT	10

const int adaMotor = 1;
const int pinEncoderA = 18;
const int pinEncoderB = 14;

int pwmMinimum = 98;
const int pwmMaximum = 200;

#define MAX_POS 5250
#define MIN_POS 0

AF_DCMotor imotorM1(adaMotor);
AF_DCMotor *motorM1;

Encoder encMotor(pinEncoderA, pinEncoderB);

static enum {
	HOMING,
	MOVING,
	IDLE
} motorMode;

long posMotorNow = 0;
long posMotorFuture = 0;
int posOvershoot;

void setup() {
	motorM1 = &imotorM1;
	pinMode(pinEncoderA, INPUT_PULLUP);
	pinMode(pinEncoderB, INPUT_PULLUP);
	Serial.begin(9600);

	/* Get a direction */
	Serial.print("Homing: ");
	motorMode = HOMING;
	motorM1->setSpeed(180);
	motorM1->run(BACKWARD);
	posMotorNow = encMotor.read() + 10;
}

int pos = -1;
int neg = 0;

void readNextPosition() {
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (pos >= 0) {
				/* Go there */
				pos *= (neg ? -1 : 1);
				if (pos > MAX_POS)
					pos = MAX_POS;
				if (pos < MIN_POS)
					pos = MIN_POS;
				Serial.print("\r\nDest: "); Serial.print(pos); Serial.print("\r\n");
				posMotorFuture = pos;
				posOvershoot = AXIS_OVERSHOOT;
				motorMode = MOVING;
			}

			/* Get a direction */
			Serial.print("\r\nPos: ");

			pos = -1;
			neg = 0;
		} else if (c == '-' && neg == 0 && pos < 0) {
				neg = 1;
			Serial.write(c);
		} else if (c >= '0' && c <= '9') {
			if (pos < 0)
				pos = 0;
			pos *= 10;
			pos += (c - '0');
			Serial.write(c);
		} else if (pos >= 0 && (c == '\b' || c == 127)) {
			Serial.print("\b \b\b");
			pos /= 10;
			if (pos == 0 && neg) {
				neg = 0;
				pos = -1;
			}
		} else {
			Serial.write('\a');
			Serial.print("BEEP[");
			Serial.print(c);
			Serial.print("] pos=");
			Serial.print(pos);
			Serial.print(", neg=");
			Serial.print(neg);
			Serial.print("\r\n");
		}
	}
}

void loop() {
	long distance;
	int direction;

	if (motorMode == HOMING) {
		static unsigned long lastms;
		unsigned long ms = millis();

		if (!lastms)
			lastms = ms + 10;

		if (lastms < ms) {
			if (encMotor.read() == posMotorNow) {
				encMotor.write(0);
				motorM1->setSpeed(0);
				motorM1->run(RELEASE);
				motorMode = IDLE;
				Serial.println("Done");
			}
			lastms = ms + 10;
			posMotorNow = encMotor.read();
		}
		return;
	}
	
	posMotorNow = encMotor.read();

	if (motorMode == IDLE) {
		readNextPosition();
		return;
	}

	distance = posMotorFuture - posMotorNow;

	if (distance >= 0) {
		direction = FORWARD;
	} else {
		direction = BACKWARD;
		distance = -distance;
	}

	if (distance == 0) {
		if (posOvershoot) {
		//	Serial.print("Overshoot: ");Serial.println(posOvershoot);
			posOvershoot--;
			motorM1->run(BRAKE);
			motorM1->setSpeed(0);
			delayMicroseconds(1000);
			return;
		}
		/* We are where we want to be */
		Serial.print("Located: ");Serial.println(posMotorNow);
		motorM1->setSpeed(0);
		motorM1->run(RELEASE);
		motorMode = IDLE;
	} else {
		int speed;

		if (distance > 50)
			speed = pwmMaximum;
		else
			speed = pwmMinimum + distance;

		Serial.println(speed);
		/* Moving ahead... */
		motorM1->setSpeed(speed);
		motorM1->run(direction);
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


