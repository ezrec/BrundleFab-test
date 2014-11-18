/* Linear control of a DC motor
 *
 *
 * Pinout:
 *
 * I2C@0x60 -> Adafruit Motorshield v2
 * P18 -> Optical encoder input A
 * P14 -> Optical encoder input B
 */

#define DEBUG_VERBOSE	1

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>

const int adaMotor = 1;
const int pinEncoderA = 18;
const int pinEncoderB = 14;

const int pwmMinimum = 50;
const int pwmMaximum = 100;

int dir = -1;
int neg = 0;
long posMotorPast = 0;
long posMotorFuture = 0;
long posMotorDelta = 0;
int rateMotor = 0;

Adafruit_MotorShield motorAFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorM1 = motorAFMS.getMotor(1);

Encoder encMotor(pinEncoderA, pinEncoderB);

void setup() {
	motorAFMS.begin();
	motorM1->setSpeed(0);
	motorM1->run(RELEASE);

	pinMode(pinEncoderA, INPUT);
	pinMode(pinEncoderB, INPUT);
	Serial.begin(115200);

	/* Get a direction */
	Serial.print("Steps: ");
}

void readNextPosition() {
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (dir >= 0) {
				/* Go there */
				dir *= (neg ? -1 : 1);
				Serial.print("\r\nGo: "); Serial.print(dir); Serial.print("\r\n");
				posMotorDelta = dir;
				posMotorFuture += posMotorDelta;
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

	if (!posMotorDelta) {
		readNextPosition();
		return;
	}

	if ((posMotorDelta > 0 && posMotorNow >= posMotorFuture) ||
	    (posMotorDelta < 0 && posMotorNow <= posMotorFuture)) {
		/* We are where we want to be */
		if (rateMotor) {
			motorM1->setSpeed(0);
			motorM1->run(RELEASE);
			Serial.print("Located @");
			Serial.println(posMotorNow);
			posMotorNow = encMotor.read();
		}
		rateMotor = 0;
		posMotorDelta = 0;
		if (posMotorNow != posMotorFuture) {
			Serial.print("Overstepped: ");
			Serial.println(posMotorNow - posMotorFuture);
			posMotorFuture = posMotorNow;
		}
	} else if (posMotorDelta) {
		int speed;
		int direction;

		speed = posMotorFuture - posMotorNow;
		if (speed < 0) {
			direction = BACKWARD;
			speed = -speed;
		} else {
			direction = FORWARD;
		}
		if (speed > pwmMaximum)
			speed = pwmMaximum;
		if (speed < pwmMinimum)
			speed = pwmMinimum;

		/* Moving ahead... */
		if (speed != rateMotor) {
			motorM1->setSpeed(speed);
			motorM1->run(direction);
			rateMotor = speed;
#if DEBUG_VERBOSE
			Serial.print("MC: speed=");
			Serial.print(rateMotor);
			Serial.print(", dir=");
			Serial.print(direction == FORWARD ? "+" : "-");
			Serial.print(", now=");
			Serial.print(posMotorNow);
			Serial.print(", future=");
			Serial.println(posMotorFuture);
#endif
		}
	}
}


