
#include <AFMotor.h>

AF_DCMotor motorX(1);
AF_DCMotor motorY(2);
AF_DCMotor motorZ(3);

void setup() {
	Serial.begin(9600);
	Serial.println("1/10 power");

	motorX.setSpeed(255);
	motorX.run(RELEASE);

	motorY.setSpeed(255);
	motorY.run(RELEASE);

	motorZ.setSpeed(255);
	motorZ.run(RELEASE);
}

void loop() {
	motorX.run(FORWARD);
	motorY.run(FORWARD);
	motorZ.run(FORWARD);
	delay(100);
	motorX.run(BACKWARD);
	motorY.run(BACKWARD);
	motorZ.run(BACKWARD);
	delay(100);
}
