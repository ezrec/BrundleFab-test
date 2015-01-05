/* Linear control of a stepper motor
 *
 * Pinout:
 *
 * M1 -> Stepper Motor (Arduino Motor Shield v2 @0x60, M1/M2)
 */

#define DEBUG_VERBOSE	0

#include <Wire.h>
#include <Adafruit_MotorShield.h>

const int adaMotor = 1;

#define MAX_POS 5250
#define MIN_POS 0

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* 200 steps/rotation (1.8 degree), 1 = M1/M2, 2 = M3/M4 */
Adafruit_StepperMotor *motorM1 = AFMS.getStepper(200, 1);

static enum {
	HOMING,
	MOVING,
	IDLE
} motorMode;

long posMotorNow = 0;
long posMotorFuture = 0;
int posOvershoot;

void setup() {
	Serial.begin(9600);

	AFMS.begin();
	/* Get a direction */
	Serial.print("Homing: ");
	motorMode = HOMING;
	/* Motor RPM  = 10 */
	motorM1->setSpeed(200);
	motorM1->step(200, FORWARD, DOUBLE);
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
		motorMode = IDLE;
		return;
	}
	
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
		/* We are where we want to be */
		Serial.print("Located: ");Serial.println(posMotorNow);
		motorMode = IDLE;
	} else {
		/* Moving ahead... */
		Serial.print("Step... ");Serial.print(distance);
		motorM1->step(distance, direction, DOUBLE);
		posMotorNow = posMotorFuture;
		Serial.println();
	}
}


