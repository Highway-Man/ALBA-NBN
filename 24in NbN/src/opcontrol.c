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

void updateVelocity(Encoder left, Encoder right) {
	double unfilteredRpmL, unfilteredRpmR;
	static long lastEncL = 0, lastEncR = 0, dEncL, dEncR, tLast = 0;
	double dt;

	dEncL = encoderGet(left) - lastEncL;
	lastEncL = encoderGet(left);
	dEncR = encoderGet(right) - lastEncR;
	lastEncR = encoderGet(right);

	dt = millis() - tLast;
	tLast = millis();

	unfilteredRpmL = 1000.0 / dt * 60.0 * dEncL / 360.0 * 5.0;
	unfilteredRpmR = 1000.0 / dt * 60.0 * dEncR / 360.0 * 5.0;

	rpmL = .75 * rpmL + .25 * unfilteredRpmL;
	rpmR = .75 * rpmR + .25 * unfilteredRpmR;
}

short flywheelCommandL, flywheelCommandR;

void tbhControllerUpdate(void *ignore) {
	long target;
	static long errorL, errorR, pErrorL = 0, pErrorR = 0;
	float kP = .016;
	static float speedL = 0, speedR = 0, speedAtZeroL = 0, speedAtZeroR = 0;
	hasCrossedL = 0, hasCrossedR = 0;

	while (1) {

		target = flywheelTargetRpm;

		updateVelocity(yellowFlywheelEncoder, greenFlywheelEncoder);

		errorL = target - rpmL;
		speedL += kP * errorL;
		errorR = target - rpmR;
		speedR += kP * errorR;

		if (speedL > 127)
			speedL = 127;
		else if (speedL < 0)
			speedL = 0;
		if (speedR > 127)
			speedR = 127;
		else if (speedR < 0)
			speedR = 0;

		if (errorL * pErrorL <= 0 && target > 0 && rpmL > 0) {
			if (!hasCrossedL) {
				speedL = 127.0 * target / 3500.0;
				hasCrossedL = 1;
			} else
				speedL = .5 * (speedL + speedAtZeroL);
			speedAtZeroL = speedL;
		}
		pErrorL = errorL;

		if (errorR * pErrorR <= 0 && target > 0 && rpmR > 0) {
			if (!hasCrossedR) {
				speedR = 127.0 * target / 3500.0;
				hasCrossedR = 1;
			} else
				speedR = .5 * (speedR + speedAtZeroR);
			speedAtZeroR = speedR;
		}

		if (errorL > 200){
			speedL = 127;
			hasCrossedL = 0;
		}
		if (errorR > 200){
			speedR = 127;
			hasCrossedR = 0;
		}

		pErrorR = errorR;

		flywheelCommandL = speedL;
		flywheelCommandR = speedR;

		if (abs(errorL) <= .02 * target && abs(errorR) <= .02 * target)
			digitalWrite(5, LOW);
		else
			digitalWrite(5, HIGH);

		printf("%d, %d, %d, %f, %f															            ", millis(), flywheelCommandR,
				flywheelCommandL, rpmR, rpmL);

		delay(20);
	}
}

int matchLoads = 0;

void rollersControl(void *ignore) {

	while (!isAutonomous()) {
		while (matchLoads==1) {
			setConveyor(127);
			while ((abs(flywheelTargetRpm - rpmL) > .01 * flywheelTargetRpm
					|| abs(flywheelTargetRpm - rpmR) > .01 * flywheelTargetRpm) && matchLoads==1) {
				delay(20);
			}
			indexFeeder(700, 0);
			delay(20);
		}
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

	if(tbhStarted == 0){
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

	while (1) {

		//toggle pneumatic actuators
		if (joystickGetDigital(1, 8, JOY_UP)) {
			digitalWrite(1, abs(digitalRead(1) - 1));
			digitalWrite(2, abs(digitalRead(2) - 1));
			while (joystickGetDigital(1, 8, JOY_UP))
				delay(20);
		}

		if (joystickGetDigital(1, 8, JOY_DOWN)) {
			matchLoads = abs(matchLoads - 1);
			while (joystickGetDigital(1, 8, JOY_DOWN))
				delay(20);
		}

		//flywheel acceleration
		if (joystickGetDigital(1, 7, JOY_UP) && !sevenIsHeld) {
			flywheelTargetRpm += 25;
			sevenIsHeld = 1;
		} else if (joystickGetDigital(1, 7, JOY_DOWN) && !sevenIsHeld) {
			flywheelTargetRpm -= 25;
			sevenIsHeld = 1;
		}
		//preset flywheel speeds
		else if (joystickGetDigital(1, 7, JOY_RIGHT))
			flywheelTargetRpm = 1500;
		else if (joystickGetDigital(1, 7, JOY_LEFT))
			flywheelTargetRpm = 1300;
		else
			sevenIsHeld = 0;

		//set flywheel motors
		motorSet(4, -flywheelCommandL);
		motorSet(5, -flywheelCommandL);
		motorSet(6, flywheelCommandR);
		motorSet(7, flywheelCommandR);

		//set drive motors
		motorSet(2, joystickGetAnalog(1, 3));
		motorSet(3, -joystickGetAnalog(1, 3));
		motorSet(8, -joystickGetAnalog(1, 2));
		motorSet(9, joystickGetAnalog(1, 2));

		//-5659 & 5141 5 rotations  -- 1082/rot
		//5701 & 5006				   1070.7/rot
		//4996 5798					   1079.4/rot	1078/rot
		//9256 11363 @ 9 rot		   1145/5/rot

		//2611, 1751 @ 100 in
		//1867, 2798->1865.333

//		printf("%d, %d; ", encoderGet(yellowDriveEncoder),
//				encoderGet(greenDriveEncoder));

		delay(20);
	}
}
