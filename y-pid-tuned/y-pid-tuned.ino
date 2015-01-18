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
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

const int adaMotor = 4;
const int pinEncoderA = 19;
const int pinEncoderB = 29;

const int pwmMinimum = 98;
const int pwmMaximum = 255;
const int pwmRange = (pwmMaximum - pwmMinimum);

#define MAX_POS (5250 - 1000)
#define MIN_POS (0 + 1000)

AF_DCMotor imotorM1(adaMotor);
AF_DCMotor *motorM1;

Encoder encMotor(pinEncoderA, pinEncoderB);

enum {
	IDLE,
	AUTOTUNE,
	MOVING
} mode;

double input = 0;
double output = 0;
double setpoint = 0;
double kp = 0.04066, ki = 0.00210, kd = 0.01;
double aTuneStep=pwmRange/3, aTuneNoise=1;
unsigned int aTuneLookBack=20;

byte m_PID_Mode;
PID m_PID = PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune m_AutoTune = PID_ATune(&input, &output);

void setup() {
	motorM1 = &imotorM1;
	pinMode(pinEncoderA, INPUT_PULLUP);
	pinMode(pinEncoderB, INPUT_PULLUP);
	Serial.begin(9600);

	/* Get a direction */
	Serial.print("Homing: ");
	motorM1->setSpeed(pwmMaximum);
	motorM1->run(BACKWARD);
	delay(1000);
	encMotor.write(-100);
	Serial.println("Done\n");

	m_PID.SetMode(AUTOMATIC);
	m_PID.SetOutputLimits(-pwmRange, pwmRange);
	mode = IDLE;
}

void pid_dump()
{
	Serial.print("PID");
	Serial.print(" Kp="); Serial.print(m_PID.GetKp(), 5);
	Serial.print(" Ki="); Serial.print(m_PID.GetKi(), 5);
	Serial.print(" Kd="); Serial.print(m_PID.GetKd(), 5);
	Serial.println();
}

int pos = -1;
int neg = 0;

int32_t posMotorNow;
int32_t posMotorFuture;

void readNextPosition() {
	if (Serial.available()) {
		int c = Serial.read();
		if (c == '?') {
			pid_dump();
			return;
		}
		if (c == 'a') {
			if (mode == AUTOTUNE) {
				m_AutoTune.Cancel();
				m_PID.SetMode(m_PID_Mode);
				pid_dump();
				mode = IDLE;
			} else {
				output = 0;
				setpoint = (MIN_POS + MAX_POS) / 2;
				m_AutoTune.SetControlType(1);
				m_AutoTune.SetNoiseBand(aTuneNoise);
				m_AutoTune.SetOutputStep(aTuneStep);
				m_AutoTune.SetLookbackSec((int)aTuneLookBack);
				m_PID_Mode = m_PID.GetMode();
				mode = AUTOTUNE;
			}
			return;
		}
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
				mode = MOVING;
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
	readNextPosition();

	posMotorNow = encMotor.read();

	setpoint = posMotorFuture;
	input = posMotorNow;

	if (mode == AUTOTUNE) {
		byte val = m_AutoTune.Runtime();

		if (val) {
			mode = IDLE;
			kp = m_AutoTune.GetKp();
			ki = m_AutoTune.GetKi();
			kd = m_AutoTune.GetKd();
			m_PID.SetTunings(kp, ki, kd);
			pid_dump();
		}
	} else {
		m_PID.Compute();
	}

	if (output == 0) {
		motorM1->run(BRAKE);
		motorM1->setSpeed(0);
		return;
	}

	if (output < 0) {
		if (0 && posMotorNow < MIN_POS) {
			motorM1->run(FORWARD);
			motorM1->setSpeed(pwmMinimum);
			return;
		}
		motorM1->run(BACKWARD);
		motorM1->setSpeed(map(output, -pwmRange, 0, pwmMaximum, pwmMinimum));
	} else {
		if (0 && posMotorNow > MAX_POS) {
			motorM1->run(BACKWARD);
			motorM1->setSpeed(pwmMinimum);
			return;
		}
		motorM1->run(FORWARD);
		motorM1->setSpeed(map(output, 0, pwmRange, pwmMinimum, pwmMaximum));
	}
}
