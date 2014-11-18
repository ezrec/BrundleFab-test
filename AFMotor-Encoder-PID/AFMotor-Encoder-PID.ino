/* Linear control of a DC motor
 *
 *
 * Pinout:
 *
 * Adafruit Motorshield v1
 * P18 -> Optical encoder input A
 * P14 -> Optical encoder input B
 */

#define DEBUG_VERBOSE	1

#include <Wire.h>
#include <AFMotor.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

const int adaMotor = 1;
const int pinEncoderA = 18;
const int pinEncoderB = 14;

const int pwmMinimum = 80;
const int pwmMaximum = 255;

int dir = -1;
int neg = 0;
long posMotorPast;
long posMotorFuture;
long posMotorDelta;
int rateMotor = 0;

AF_DCMotor imotorM1(adaMotor);
AF_DCMotor *motorM1;

Encoder encMotor(pinEncoderA, pinEncoderB);

double pidM1Desired, pidM1Input, pidM1Output;

const double pidKpM1 = 0.01782;
const double pidKiM1 = 0.00085;
const double pidKdM1 = 0.00000;
bool tuned = false;

PID pidM1(&pidM1Input, &pidM1Output, &pidM1Desired, pidKpM1, pidKiM1, pidKdM1, DIRECT);
PID_ATune pidATuneM1(&pidM1Input, &pidM1Output);

void motor_home(AF_DCMotor *m, Encoder *e)
{
	long pos, pos_new;

	if (tuned) {
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
	}

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

void setup() {
	Serial.begin(115200);

	motorM1 = &imotorM1;
	motorM1->setSpeed(0);
	motorM1->run(RELEASE);

	pinMode(pinEncoderA, INPUT);
	pinMode(pinEncoderB, INPUT);

	motor_home(motorM1, &encMotor);

	/* Initialize the PID */
	pidM1.SetOutputLimits(-1.0, 1.0);
	pidM1.SetMode(AUTOMATIC);

	pidATuneM1.SetOutputStep(0.1);

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

unsigned long ms_last;

void loop() {
	long posMotorNow = encMotor.read();
	bool delta;

	if (tuned && readNextPosition())
		return;

	pidM1Desired= posMotorFuture;
	pidM1Input = posMotorNow;

	unsigned long ms_now = millis();
	if ((ms_now - ms_last) < 10) {
		return;
	}
	ms_last = ms_now;

	if (!tuned) {
		delta = (pidATuneM1.Runtime() == 0);
		tuned = !delta;
		if (tuned) {
			Serial.print("Kp=");Serial.print(pidATuneM1.GetKp());
			Serial.print("Ki=");Serial.print(pidATuneM1.GetKi());
			Serial.print("Kd=");Serial.println(pidATuneM1.GetKd());
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
#if DEBUG_VERBOSE
		static int nsteps = 0;
		nsteps++;
		if (nsteps == 100) {
			Serial.print("input=");Serial.print(pidM1Input);
			Serial.print(", desired=");Serial.print(pidM1Desired);
			Serial.print(", output=");Serial.println(pidM1Output);
			nsteps = 0;
		}
#endif
		if (pidM1Output == 0) {
			motorM1->run(RELEASE);
		} else if (pidM1Output < 0) {
			motorM1->setSpeed(-pidM1Output*(pwmMaximum-pwmMinimum) + pwmMinimum);
			motorM1->run(BACKWARD);
		} else {
			motorM1->setSpeed(pidM1Output*(pwmMaximum-pwmMinimum) + pwmMinimum);
			motorM1->run(FORWARD);
		}
	}
}


