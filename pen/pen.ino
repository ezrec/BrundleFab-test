/* Pen up/down
 *
 *
 * Pinout:
 *
 * M3 -> DC Motor control (Adafruit Motor Shield v1)
 */

#define DEBUG_VERBOSE	0

#include <Wire.h>
#include <AFMotor.h>

const int adaMotor = 3;

const int pwm = 255;

AF_DCMotor imotorM1(adaMotor);
AF_DCMotor *motorM1;

void setup() {
	motorM1 = &imotorM1;
	Serial.begin(9600);
}

void loop() {
	Serial.println("Pendown");
	motorM1->setSpeed(pwm);
	motorM1->run(FORWARD);
	delay(30);
	motorM1->run(RELEASE);
	delay(1000);
	Serial.println("Penup");
	motorM1->setSpeed(pwm);
	motorM1->run(BACKWARD);
	delay(20);
	motorM1->run(RELEASE);
	for (;;);
}
