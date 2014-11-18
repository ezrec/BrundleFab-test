/* Linear control of a DC motor
 *
 *
 * Pinout:
 *
 * I2C@0x60 -> Adafruit Motorshield v2
 * P18 -> Optical encoder input A
 * P14 -> Optical encoder input B
 */

#define DEBUG_VERBOSE	0

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

const int adaMotor = 1;
const int pinEncoderA = 18;
const int pinEncoderB = 14;

const int pwmMinimum = 50;
const int pwmMaximum = 255;

int dir = -1;
int neg = 0;
long posMotorPast;
long posMotorFuture;
long posMotorDelta;
int rateMotor = 0;

Adafruit_MotorShield motorAFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motorM1 = motorAFMS.getMotor(1);

Encoder encMotor(pinEncoderA, pinEncoderB);

double pidM1Desired, pidM1Input, pidM1Output;

const double pidKpM1 = 0.0100;
const double pidKiM1 = 0.0001;
const double pidKdM1 = 0.0001;
PID pidM1(&pidM1Input, &pidM1Output, &pidM1Desired, pidKpM1, pidKiM1, pidKdM1, DIRECT);
PID_ATune pidATuneM1(&pidM1Input, &pidM1Output);

void motor_home(Adafruit_DCMotor *m, Encoder *e)
{
	long pos, pos_new;

	// Home the motor
	motorM1->setSpeed(pwmMinimum+(pwmMaximum-pwmMinimum)/4);
	motorM1->run(BACKWARD);

	// Find the minimum position
	pos_new = e->read();
	do {
		pos = pos_new;
		delay(100);
		pos_new = e->read();
	} while (pos_new != pos);

	motorM1->setSpeed(0);
	motorM1->run(RELEASE);

	pidM1Desired = 0;
	pidM1Input = 0;
	pidM1Output = 0;
	e->write(0);

	posMotorPast = 0;
	posMotorFuture = 0;
	posMotorDelta = 0;
}

bool tuned;

void setup() {
	Serial.begin(115200);

	motorAFMS.begin();
	motorM1->setSpeed(0);
	motorM1->run(RELEASE);

	pinMode(pinEncoderA, INPUT);
	pinMode(pinEncoderB, INPUT);

	motor_home(motorM1, &encMotor);

	/* Initialize the PID */
	pidM1.SetOutputLimits(-(pwmMaximum-pwmMinimum), (pwmMaximum-pwmMinimum));
	pidM1.SetMode(AUTOMATIC);

	pidATuneM1.SetOutputStep(10);

	tuned = true;

	/* Get a direction */
	Serial.print("Steps: ");
}

bool readNextPosition() {
	if (Serial.available()) {
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
		return true;
	}

	return false;
}

void loop() {
	long posMotorNow = encMotor.read();
	bool delta;

	if (readNextPosition())
		return;

	pidM1Desired= posMotorFuture;
	pidM1Input = posMotorNow;

	if (!tuned) {
		delta = (pidATuneM1.Runtime() == 0);
		tuned = !delta;
		if (tuned) {
			Serial.print("Kp=");Serial.print(pidATuneM1.GetKp()*1000);
			Serial.print("Ki=");Serial.print(pidATuneM1.GetKi()*1000);
			Serial.print("Kd=");Serial.println(pidATuneM1.GetKd()*1000);
			pidM1.SetTunings(pidATuneM1.GetKp(),
					 pidATuneM1.GetKi(),
					 pidATuneM1.GetKd());
			pidM1.SetControllerDirection(DIRECT);
			motorM1->run(RELEASE);
			for (;;);
		}
	} else {
		delta = pidM1.Compute();
	}

	if (delta) {
//Serial.print("input=");Serial.print(pidM1Input);
//Serial.print(", desired=");Serial.print(pidM1Desired);
//Serial.print(", output=");Serial.println(pidM1Output);
		if (pidM1Output == 0) {
			motorM1->run(RELEASE);
		} else if (pidM1Output < 0) {
			motorM1->setSpeed(-pidM1Output + pwmMinimum);
			motorM1->run(BACKWARD);
		} else {
			motorM1->setSpeed(pidM1Output + pwmMinimum);
			motorM1->run(FORWARD);
		}
	}
}


