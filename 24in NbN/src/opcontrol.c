/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

double rpmL = 0, rpmR = 0;
//flywheel velocity
void updateVelocity(Encoder left, Encoder right) {
	double unfilteredRpmL, unfilteredRpmR;
	static long lastEncL = 0, lastEncR = 0, dEncL, dEncR, tLast = 0;
	double dt;

	//calc change in L encoder position
	dEncL = encoderGet(left) - lastEncL;
	//save position
	lastEncL = encoderGet(left);
	//calc change in R encoder position
	dEncR = encoderGet(right) - lastEncR;
	//save last position
	lastEncR = encoderGet(right);

	//calc change in time
	dt = millis() - tLast;
	tLast = millis();

	//calculate rpms from changes
	unfilteredRpmL = 1000.0 / dt * 60.0 * dEncL / 360.0 * 5.0;
	unfilteredRpmR = 1000.0 / dt * 60.0 * dEncR / 360.0 * 5.0;

	//apply IIR filter
	rpmL = .60 * rpmL + .40 * unfilteredRpmL;
	rpmR = .60 * rpmR + .40 * unfilteredRpmR;
}

const unsigned int flywheelLeftLinearControl[128] = { 0, 14, 15, 16, 16, 17, 17,
		18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 22,
		23, 23, 23, 23, 23, 24, 24, 24, 24, 25, 25, 25, 25, 26, 26, 26, 26, 27,
		27, 27, 28, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32,
		33, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 37, 37, 37, 38, 38, 39, 39,
		40, 40, 41, 42, 42, 43, 43, 44, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51,
		52, 52, 53, 54, 55, 56, 57, 59, 60, 61, 62, 63, 65, 66, 68, 70, 72, 73,
		74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 86, 127 };

short flywheelCommandL, flywheelCommandR;
//update flywheel velocity controller
void tbhControllerUpdate(void *ignore) {
	long target;
	static long errorL, errorR, pErrorL = 0, pErrorR = 0;
	float kP = .016;
	static float speedL = 0, speedR = 0, speedAtZeroL = 0, speedAtZeroR = 0;
	hasCrossedL = 0, hasCrossedR = 0;

	while (1) {

		//save target and update velocity
		target = flywheelTargetRpm;
		updateVelocity(yellowFlywheelEncoder, greenFlywheelEncoder);

		//calculate v error for both flywheels
		//integrate error
		errorL = target - rpmL;
		speedL += kP * errorL;
		errorR = target - rpmR;
		speedR += kP * errorR;

		//implements take back half when error crosses zero
		if (errorL * pErrorL <= 0 && target > 0 && rpmL > 0) {
			if (!hasCrossedL) {
				speedL = 127.0 * target / 2100 * 6800 / powerLevelMain();
				hasCrossedL = 1;
			} else
				speedL = .5 * (speedL + speedAtZeroL);
			speedAtZeroL = speedL;
		}
		pErrorL = errorL;

		//same for other side
		if (errorR * pErrorR <= 0 && target > 0 && rpmR > 0) {
			if (!hasCrossedR) {
				speedR = (127.0 * target / 2040 * 7000 / powerLevelMain()) + 5;
				hasCrossedR = 1;
			} else
				speedR = .5 * (speedR + speedAtZeroR);
			speedAtZeroR = speedR;
		}

		//set to full speed after firing a ball
		if (errorL > 200) {
			speedL = 127;
			hasCrossedL = 0;
		}
		if (errorR > 200) {
			speedR = 127;
			hasCrossedR = 0;
		}

		//cap at max speed
		//don't go backwards
		if (speedL > 127)
			speedL = 127;
		else if (speedL < 0)
			speedL = 0;
		if (speedR > 127)
			speedR = 127;
		else if (speedR < 0)
			speedR = 0;

		//save values
		pErrorR = errorR;

		flywheelCommandL = speedL;
		flywheelCommandR = speedR;

		//light LED when we are at target velocity
		if (abs(errorL) <= .02 * target && abs(errorR) <= .02 * target)
			digitalWrite(5, LOW);
		else
			digitalWrite(5, HIGH);

		//print debug data to terminal
		printf("%d, %d, %d, %f, %f															            ", millis(),
				motorGet(6), -motorGet(4), rpmR, rpmL);

		delay(20);
	}
}

int matchLoads = 0;
//controls both intake rollers in tele op
void rollersControl(void *ignore) {

	while (!isAutonomous()) {
		//autonomatically run intake and feeder
		while (matchLoads == 1) {
			setConveyor(127);
			while ((abs(flywheelTargetRpm - rpmL) > .01 * flywheelTargetRpm
					|| abs(flywheelTargetRpm - rpmR) > .01 * flywheelTargetRpm)
					&& matchLoads == 1) {
				delay(20);
			}
			indexFeeder(700, 0);
			delay(20);
		}
		//manual
		//set feed motor
		if (joystickGetDigital(1, 6, JOY_UP))
			motorSet(10, 127);
		else if (joystickGetDigital(1, 6, JOY_DOWN))
			motorSet(10, -127);
		else
			motorSet(10, 0);

		//set conveyor motor on/off/reverse
		if (joystickGetDigital(1, 5, JOY_UP))
			motorSet(1, 127);
		else if (joystickGetDigital(1, 5, JOY_DOWN))
			motorSet(1, -127);
		else
			motorSet(1, 0);

		delay(20);
	}
}

int done = 0;

void operatorControl() {

	//autonomous();

	static int sevenIsHeld = 0;

	//start tasks we haven't started yet
	if (tbhStarted == 0) {
		taskCreate(tbhControllerUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
				TASK_PRIORITY_DEFAULT);
		tbhStarted = 1;
	}

	taskCreate(rollersControl, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);

//	matchLoads = 1;

//	flywheelTargetRpm = 1475;
//	motorSet(10,127);
//	motorSet(1,127);
//	while(1){
//		//set flywheel motors
//		motorSet(4, -flywheelCommandL);
//		motorSet(5, -flywheelCommandL);
//		motorSet(6, flywheelCommandR);
//		motorSet(7, flywheelCommandR);
//		delay(20);
//	}

//	for (int i = 0; i < 128; i++) {
//		setFlywheels(i, i);
//		delay(2000);
//		updateVelocity(yellowFlywheelEncoder, greenFlywheelEncoder);
//		printf("%d, %4.2f   												     	 		 	  ", i, rpmL);
//		delay(50);
//		printf("%d, %4.2f   												     	 		 	  ", i, rpmR);
//		delay(50);
//	}

	while (1) {
//		flywheelTargetRpm = 1300;
//		motorSet(1, 127);
//		motorSet(10, 127);

		//toggle pneumatic actuators
		if (joystickGetDigital(1, 8, JOY_UP)) {
			digitalWrite(1, abs(digitalRead(1) - 1));
			while (joystickGetDigital(1, 8, JOY_UP))
				delay(20);
		}

		//brake
		if (joystickGetDigital(1, 8, JOY_LEFT))
			digitalWrite(2, HIGH);
		else
			digitalWrite(2, LOW);

		//activate automatic firing
		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			matchLoads = abs(matchLoads - 1);
			while (joystickGetDigital(1, 8, JOY_DOWN))
				delay(20);
		}

		//flywheel acceleration
		if (joystickGetDigital(1, 7, JOY_UP)) {
			if (!sevenIsHeld) {
				flywheelTargetRpm += 25;
				sevenIsHeld = 1;
			}
		} else if (joystickGetDigital(1, 7, JOY_DOWN)) {
			if (!sevenIsHeld) {
				flywheelTargetRpm -= 25;
				sevenIsHeld = 1;
			}
		}
		//preset flywheel speeds
		else if (joystickGetDigital(1, 7, JOY_RIGHT))
			flywheelTargetRpm = 1500;
		else if (joystickGetDigital(1, 7, JOY_LEFT))
			flywheelTargetRpm = 1300;
		else
			sevenIsHeld = 0;

		//set flywheel motors
		motorSet(4, -flywheelLeftLinearControl[flywheelCommandL]);
		motorSet(5, -flywheelLeftLinearControl[flywheelCommandL]);
		motorSet(6, flywheelLeftLinearControl[flywheelCommandR]);
		motorSet(7, flywheelLeftLinearControl[flywheelCommandR]);

		//set drive motors
		motorSet(2, joystickGetAnalog(1, 3));
		motorSet(3, -joystickGetAnalog(1, 3));
		motorSet(8, -joystickGetAnalog(1, 2));
		motorSet(9, joystickGetAnalog(1, 2));

//		print debug data to terminal
//		printf("%d, %d; ", encoderGet(yellowDriveEncoder),
//				encoderGet(greenDriveEncoder));

		delay(20);
	}
}
