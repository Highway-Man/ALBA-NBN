/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
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
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */

#define ticksPerIn	28.10 //25.87 //26.23 //27.43 // 27.64
#define ticksPerDeg	3.90 //471 163 // 392 253
short sign(int var) {
	if (var > 0)
		return 1;
	else if (var < 0)
		return -1;
	else
		return 0;
}

void setFlywheels(short commandL, short commandR) {
	motorSet(4, -commandL);
	motorSet(5, -commandL);
	motorSet(6, commandR);
	motorSet(7, commandR);
}
void accelerateFlywheels(short target) {
	short command = 0;
	while (command < target) {
		command++;
		setFlywheels(command, command);
		delay(50);
	}
	while (command > target) {
		command--;
		setFlywheels(command, command);
		delay(50);
	}
}

void setConveyor(short command) {
	motorSet(1, command);
}

void setFeeder(short command) {
	motorSet(10, command);
}
void indexFeeder(int timeRunning, int timeStopped) {
	setFeeder(127);
	delay(timeRunning);
	setFeeder(0);
	delay(timeStopped);
}

void setDrive(short left, short right) {
	motorSet(2, left);
	motorSet(3, -left);
	motorSet(8, -right);
	motorSet(9, right);
}

void deployRamps() {
	digitalWrite(1, HIGH);
	digitalWrite(2, HIGH);
}

void flywheelTargetRpmSet(long rpm) {
	flywheelTargetRpm = rpm;
}

void waitForFlywheels(float percentError, long target) {
	static long pRpmL[2] = { 0, 0 };
	static long pRpmR[2] = { 0, 0 };
	while (abs(target - rpmL) > percentError * target //|| abs(target - pRpmL[1]) > percentError * target
	|| abs(target - rpmR) > percentError * target //|| abs(target - pRpmR[1]) > percentError * target
	) {
		pRpmL[1] = pRpmL[0];
		pRpmL[0] = rpmL;
		pRpmR[1] = pRpmR[0];
		pRpmR[0] = rpmR;
		delay(20);
	}
}

long baseEncoderLGet() {
	return encoderGet(yellowDriveEncoder);
}
long baseEncoderRGet() {
	return 1.5 * encoderGet(greenDriveEncoder);
}
long baseEncAveGet() {
	return .5 * (baseEncoderLGet() + baseEncoderRGet());
}
long absBaseEncAveGet() {
	return .5 * (abs(baseEncoderLGet()) + abs(baseEncoderRGet()))
			* sign(baseEncoderRGet());
}
void baseEncReset() {
	encoderReset(yellowDriveEncoder);
	encoderReset(greenDriveEncoder);
}

void pControllerUpdate(void *ignore) {
	long errorDist, errorDiff, distanceProportion, differenceProportion;
	long distanceProportionMax = 110;
	float kPDist = .0223, kPDiff = .8;
	float minCommand, minCommandSt = 19;
	short brakingPower = -1, kAccel = 3;
	static short pBaseCommandL = 0, pBaseCommandR = 0;
	static long pBaseEncoderL = 0, pBaseEncoderR = 0;
	static long pBaseTargetPosition;

	float kPTurn = .04;
	float minCommandTurn, minCommandTurnSt = 35;

	minCommand = minCommandSt;
	minCommandTurn = minCommandTurnSt;

	while (isAutonomous()) {

		if (baseTargetPosition != pBaseTargetPosition) {
			minCommandTurn = minCommandTurnSt;
			minCommand = minCommandSt;
		}

		if (dir == turn) {

			errorDist = baseTargetPosition - absBaseEncAveGet();
			distanceProportion = kPTurn * errorDist;

			if (abs(distanceProportion)
					> distanceProportionMax - minCommandTurn)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommandTurn);

			errorDiff = baseEncoderLGet() + baseEncoderRGet();
			differenceProportion = kPDiff * errorDiff;

			baseCommandL = minCommandTurn * -1 * sign(errorDist)
					- distanceProportion - differenceProportion;
			baseCommandR = minCommandTurn * sign(errorDist) + distanceProportion
					- differenceProportion;

			if (abs(errorDist) < 15) {
				if (baseEncoderLGet() > pBaseEncoderL)
					baseCommandL = brakingPower;
				else if (baseEncoderLGet() < pBaseEncoderL)
					baseCommandL = -1 * brakingPower;
				else
					baseCommandL = -1 * sign(pBaseCommandL) * brakingPower;
				if (baseEncoderRGet() > pBaseEncoderR)
					baseCommandR = brakingPower;
				else if (baseEncoderRGet() < pBaseEncoderR)
					baseCommandR = -1 * brakingPower;
				else
					baseCommandR = -1 * sign(pBaseCommandR) * brakingPower;
			}

			if (abs(abs(baseCommandL) - abs(pBaseCommandL)) > 21)
				baseCommandL = pBaseCommandL + kAccel * sign(baseCommandL);
			if (abs(abs(baseCommandR) - abs(pBaseCommandR)) > 21)
				baseCommandR = pBaseCommandR + kAccel * sign(baseCommandR);

			if ((abs(baseCommandL) >= minCommandTurn
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommandTurn
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommandTurn += .1;

			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();

		} else {

			errorDist = baseTargetPosition - baseEncAveGet();
			distanceProportion = kPDist * errorDist;

			if (abs(distanceProportion) > distanceProportionMax - minCommand)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommand);

			errorDiff = baseEncoderLGet() - baseEncoderRGet();

			differenceProportion = kPDiff * errorDiff;

			baseCommandL = minCommand * sign(errorDist) + distanceProportion
					- differenceProportion;
			baseCommandR = minCommand * sign(errorDist) + distanceProportion
					+ differenceProportion;
			if (abs(errorDist) < 15) {
				if (baseEncoderLGet() > pBaseEncoderL)
					baseCommandL = brakingPower;
				else if (baseEncoderLGet() < pBaseEncoderL)
					baseCommandL = -1 * brakingPower;
				else
					baseCommandL = -1 * sign(pBaseCommandL) * brakingPower;
				if (baseEncoderRGet() > pBaseEncoderR)
					baseCommandR = brakingPower;
				else if (baseEncoderRGet() < pBaseEncoderR)
					baseCommandR = -1 * brakingPower;
				else
					baseCommandR = -1 * sign(pBaseCommandR) * brakingPower;
			}

			if (abs(abs(baseCommandL) - abs(pBaseCommandL)) > 21)
				baseCommandL = pBaseCommandL + kAccel * sign(baseCommandL);
			if (abs(abs(baseCommandR) - abs(pBaseCommandR)) > 21)
				baseCommandR = pBaseCommandR + kAccel * sign(baseCommandR);

			if ((abs(baseCommandL) >= minCommand
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommand
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommand += .1;

			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();
		}
		//printf("%d, %d;  ", errorDist, baseCommandR);
		pBaseTargetPosition = baseTargetPosition;
		delay(20);
	}
}

void baseTargetPositionStraightSet(long inches) {
	baseTargetPosition = ticksPerIn * inches;
	dir = straight;
}
void baseTargetPositionTurnSet(long degrees) {
	baseTargetPosition = ticksPerDeg * degrees;
	dir = turn;
}

void waitForBase(float margin) {
	static long pBaseEncAve[5] = { 0, 0, 0, 0, 0 };
	if (dir == turn) {
		while (abs(baseTargetPosition - absBaseEncAveGet()) > margin
				|| absBaseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = absBaseEncAveGet();
			delay(20);
		}
	} else {
		while (abs(baseTargetPosition - baseEncAveGet()) > margin
				|| baseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = baseEncAveGet();
			delay(20);
		}
	}
}

void waitForBaseTimeout(float margin, long timeout) {
	static long pBaseEncAve[5] = { 0, 0, 0, 0, 0 };
	if (dir == turn) {
		while (abs(baseTargetPosition - absBaseEncAveGet()) > margin
				|| absBaseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = absBaseEncAveGet();
			if (timeout <= 0)
				return;
			timeout -= 20;
			delay(20);
		}
	} else {
		while (abs(baseTargetPosition - baseEncAveGet()) > margin
				|| baseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = baseEncAveGet();
			if (timeout <= 0)
				return;
			timeout -= 20;
			delay(20);
		}
	}
}

void motorsUpdate(void *ignore) {
	while (isAutonomous()) {
		setFlywheels(flywheelCommandL, flywheelCommandR);
		setDrive(baseCommandL, baseCommandR);
		delay(20);
	}
}

void scorePreloads() {
	accelerateFlywheels(65);
	delay(2000);
	for (short i = 0; i < 4; i++)
		indexFeeder(1500, 3000);
	accelerateFlywheels(0);
}

void startUpdaterTasks() {

	printf("/n \n %d /n \n", taskGetState(tbhControllerUpdate));

	if (tbhStarted == 0) {
		taskCreate(tbhControllerUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
				TASK_PRIORITY_DEFAULT);
		tbhStarted = 1;
	}
	taskCreate(pControllerUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);
	taskCreate(motorsUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);
}

void suspendUpdaterTasks() {
	taskSuspend(tbhControllerUpdate);
	taskSuspend(pControllerUpdate);
	taskSuspend(motorsUpdate);
}

void scorePreloadsSmart() {
	flywheelTargetRpm = 1500;
	waitForFlywheels(.01, flywheelTargetRpm);
	setConveyor(127);
	delay(500);
	waitForFlywheels(.01, flywheelTargetRpm);
	indexFeeder(600, 0);
	waitForFlywheels(.01, flywheelTargetRpm);
	indexFeeder(600, 0);
	waitForFlywheels(.01, flywheelTargetRpm);
	indexFeeder(600, 0);
	waitForFlywheels(.01, flywheelTargetRpm);
	indexFeeder(900, 500);
	flywheelTargetRpm = 0;
	setConveyor(0);
}

void repositionForStacks() {
	baseEncReset();
	baseTargetPositionStraightSet(0);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionTurnSet(-135);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionStraightSet(-26);
	waitForBaseTimeout(15, 2000);
	baseEncReset();
	baseTargetPositionStraightSet(9);
	waitForBase(15);
	baseEncReset();
}

void pickUpScore2Stacks() {
	baseEncReset();
	baseTargetPositionTurnSet(-15);
	waitForBase(15);
	baseEncReset();
	setConveyor(127);
	baseTargetPositionStraightSet(60);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionTurnSet(100);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionStraightSet(20);
	flywheelTargetRpmSet(1350);
	waitForBaseTimeout(15, 5000);
	setConveyor(-127);
	delay(800);
	setConveyor(127);
	delay(1300);
	setConveyor(-127);
	delay(900);
	setConveyor(127);
	baseEncReset();
	baseTargetPositionTurnSet(29);
	waitForBase(15);
	waitForFlywheels(.02, flywheelTargetRpm);
	for (int i = 0; i < 3; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		waitForFlywheels(.02, flywheelTargetRpm);
	}
	baseEncReset();
	baseTargetPositionTurnSet(-29);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionStraightSet(24);
	waitForBaseTimeout(15, 3000);
	baseTargetPositionStraightSet(6);
	waitForBaseTimeout(15, 2000);
	setConveyor(-127);
	delay(1300);
	setConveyor(127);
	delay(200);
	baseEncReset();
	baseTargetPositionTurnSet(28);
	waitForBase(15);
	waitForFlywheels(.02, flywheelTargetRpm);
	for (int i = 0; i < 8; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		waitForFlywheels(.02, flywheelTargetRpm);
	}
}

long time;
void progSkills() {
	time = millis();
	baseTargetPositionStraightSet(-112);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionTurnSet(-90);
	waitForBase(15);
	baseEncReset();
	flywheelTargetRpmSet(1315); //1525 for full field
	baseTargetPositionStraightSet(40);
	waitForBaseTimeout(15, 2500);
	baseEncReset();
	baseTargetPositionStraightSet(-18);
	waitForBase(15);
	setConveyor(127);
	baseEncReset();
	baseTargetPositionTurnSet(5);
	waitForBase(15);
	waitForFlywheels(.02, flywheelTargetRpm);
	for (int i = 0; i < 38; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		while (abs(flywheelTargetRpm - rpmL) > .02 * flywheelTargetRpm
				|| abs(flywheelTargetRpm - rpmR) > .02 * flywheelTargetRpm) {
			if (millis() - time > 51000)
				break;
			delay(20);
		}
		if (millis() - time > 51000)
			break;
	}
	flywheelTargetRpmSet(0);
	setConveyor(0);
	baseEncReset();
	baseTargetPositionStraightSet(0);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionTurnSet(84);
	waitForBase(15);
	baseEncReset();
	baseTargetPositionStraightSet(-24);
	waitForBaseTimeout(15, 1500);
	baseEncReset();
	baseTargetPositionStraightSet(0);
	deployRamps();
}

void autonomous() {
	startUpdaterTasks();
	if (autonomousMode == skills)
		progSkills();
	else if (autonomousMode == stacks) {
		scorePreloadsSmart();
		repositionForStacks();
		pickUpScore2Stacks();
		flywheelTargetRpmSet(0);
		setConveyor(0);
	}
	else if(autonomousMode == preloads)
		scorePreloadsSmart();


	while (1) {
		//printf("%d, %d;  ", baseEncoderLGet(), baseEncoderRGet());
		delay(20);
	}
}
