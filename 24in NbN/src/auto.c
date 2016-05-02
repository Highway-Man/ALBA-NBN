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

//conversion factors for base encoders
#define ticksPerIn	28.10 //25.87 //26.23 //27.43 // 27.64
#define ticksPerDeg	3.90 //471 163 // 392 253
//get sign of variable
short sign(int var) {
	if (var > 0)
		return 1;
	else if (var < 0)
		return -1;
	else
		return 0;
}

//set flywheel motors
void setFlywheels(short commandL, short commandR) {
	motorSet(4, -commandL);
	motorSet(5, -commandL);
	motorSet(6, commandR);
	motorSet(7, commandR);
}
//slowly accelerate flywheels - not used anymore
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

//set conveyor motor
void setConveyor(short command) {
	motorSet(1, command);
}
//set feeder motor
void setFeeder(short command) {
	motorSet(10, command);
}
//index feeder automatically
void indexFeeder(int timeRunning, int timeStopped) {
	setFeeder(127);
	delay(timeRunning);
	setFeeder(0);
	delay(timeStopped);
}
//set drive motors
void setDrive(short left, short right) {
	motorSet(2, left);
	motorSet(3, -left);
	motorSet(8, -right);
	motorSet(9, right);
}
//release pneumatic cylinders to deploy ramp
void deployRamps() {
	digitalWrite(1, HIGH);
	digitalWrite(2, HIGH);
}
//set the flywheels' target rpm
void flywheelTargetRpmSet(long rpm) {
	flywheelTargetRpm = rpm;
}
//wait for the flywheels to reach their target rpm
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

//return left base encoder position
long baseEncoderLGet() {
	return 1.5 * encoderGet(yellowDriveEncoder);
}//return rught base encoder position
long baseEncoderRGet() {
	return 1.5 * encoderGet(greenDriveEncoder);
}//return average of base encoder positions
long baseEncAveGet() {
	return .5 * (baseEncoderLGet() + baseEncoderRGet());
}//return average of absolute values of base encoders with sign of right encoder
long absBaseEncAveGet() {
	return .5 * (abs(baseEncoderLGet()) + abs(baseEncoderRGet()))
			* sign(baseEncoderRGet());
}//reset base encoders to 0
void baseEncReset() {
	encoderReset(yellowDriveEncoder);
	encoderReset(greenDriveEncoder);
}

//position controller for base in autonomous
void pControllerUpdate(void *ignore) {
	long errorDist, errorDiff, distanceProportion, differenceProportion;
	long distanceProportionMax = 110;
	float kPDist = .0223, kPDiff = .8;
	float minCommand, minCommandSt = 22;
	short brakingPower = -1, kAccel = 3;
	static short pBaseCommandL = 0, pBaseCommandR = 0;
	static long pBaseEncoderL = 0, pBaseEncoderR = 0;
	static long pBaseTargetPosition;

	float kPTurn = .04;
	float minCommandTurn, minCommandTurnSt = 33;

	minCommand = minCommandSt;
	minCommandTurn = minCommandTurnSt;

	while (isAutonomous()) {
		//reset minimum powers if given a new target
		if (baseTargetPosition != pBaseTargetPosition) {
			minCommandTurn = minCommandTurnSt;
			minCommand = minCommandSt;
		}

		//turning
		if (dir == turn) {
			//calculate distance from target
			errorDist = baseTargetPosition - absBaseEncAveGet();
			//scale
			distanceProportion = kPTurn * errorDist;

			//cap distance proportion
			if (abs(distanceProportion)
					> distanceProportionMax - minCommandTurn)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommandTurn);

			//calculate difference between sides
			errorDiff = baseEncoderLGet() + baseEncoderRGet();
			differenceProportion = kPDiff * errorDiff;

			//turn based on degrees yet to turn, difference between sides and minimum power
			baseCommandL = minCommandTurn * -1 * sign(errorDist)
					- distanceProportion - differenceProportion;
			baseCommandR = minCommandTurn * sign(errorDist) + distanceProportion
					- differenceProportion;

			//apply braking power when we get close
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

			//limit acceleration
			if (abs(abs(baseCommandL) - abs(pBaseCommandL)) > 21)
				baseCommandL = pBaseCommandL + kAccel * sign(baseCommandL);
			if (abs(abs(baseCommandR) - abs(pBaseCommandR)) > 21)
				baseCommandR = pBaseCommandR + kAccel * sign(baseCommandR);

			//increase minimum power if we stall
			if ((abs(baseCommandL) >= minCommandTurn
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommandTurn
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommandTurn += .1;

			//save previous values
			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();

		//else we are driving straight & not turning
		} else {
			//calculate distance yet to drive & scale it
			errorDist = baseTargetPosition - baseEncAveGet();
			distanceProportion = kPDist * errorDist;

			//limit max
			if (abs(distanceProportion) > distanceProportionMax - minCommand)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommand);

			//calc difference between sides so we know we are driving straight
			errorDiff = baseEncoderLGet() - baseEncoderRGet();
			differenceProportion = kPDiff * errorDiff;

			//sum components
			baseCommandL = minCommand * sign(errorDist) + distanceProportion
					- differenceProportion;
			baseCommandR = minCommand * sign(errorDist) + distanceProportion
					+ differenceProportion;

			//reverse motors when we are close
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

			//limit acceleration
			if (abs(abs(baseCommandL) - abs(pBaseCommandL)) > 21)
				baseCommandL = pBaseCommandL + kAccel * sign(baseCommandL);
			if (abs(abs(baseCommandR) - abs(pBaseCommandR)) > 21)
				baseCommandR = pBaseCommandR + kAccel * sign(baseCommandR);

			if ((abs(baseCommandL) >= minCommand
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommand
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommand += .04;

			//store previous values
			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();
		}
		//print debug data
		//printf("%d, %d;  ", errorDist, baseCommandR);
		pBaseTargetPosition = baseTargetPosition;
		delay(20);
	}
}

//set distance to drive in inches
void baseTargetPositionStraightSet(long inches) {
	baseTargetPosition = ticksPerIn * inches;
	dir = straight;
}//set degrees to turn
void baseTargetPositionTurnSet(long degrees) {
	baseTargetPosition = ticksPerDeg * degrees;
	dir = turn;
}

//wait for base to reach target position
void waitForBase(float margin) {
	static long pBaseEncAve[5] = { 0, 0, 0, 0, 0 };
	//for turning
	if (dir == turn) {
		while (abs(baseTargetPosition - absBaseEncAveGet()) > margin
				|| absBaseEncAveGet() != pBaseEncAve[4]) {
			//make sure we stabilize
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = absBaseEncAveGet();
			delay(20);
		}
	} else { //for driving straight
		while (abs(baseTargetPosition - baseEncAveGet()) > margin
				|| baseEncAveGet() != pBaseEncAve[4]) {
			//make sure we stabilize
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = baseEncAveGet();
			delay(20);
		}
	}
}

//same as above, but quit waiting if something goes wrong
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

//update flywheel and drive motors
void motorsUpdate(void *ignore) {
	while (isAutonomous()) {
		setFlywheels(flywheelCommandL, flywheelCommandR);
		setDrive(baseCommandL, baseCommandR);
		delay(20);
	}
}

//start tasks to update motors commands, velocity, and position
void startUpdaterTasks() {

	printf("/n \n %d /n \n", taskGetState(tbhControllerUpdate));

	//start velocity controller if we haven't already
	if (tbhStarted == 0) {
		taskCreate(tbhControllerUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
				TASK_PRIORITY_DEFAULT);
		tbhStarted = 1;
	}//start base position controller
	taskCreate(pControllerUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);
	//start motors updater
	taskCreate(motorsUpdate, TASK_DEFAULT_STACK_SIZE, NULL,
			TASK_PRIORITY_DEFAULT);
}

//stop running tasks
void suspendUpdaterTasks() {
	taskSuspend(tbhControllerUpdate);
	taskSuspend(pControllerUpdate);
	taskSuspend(motorsUpdate);
}

long time;

//autonomous routine to drive across field to low goal and score preloads
void ram() {
	flywheelTargetRpmSet(1300);
	setConveyor(127);
	baseEncReset();
	//drive to low goal
	baseTargetPositionStraightSet(-132);
	waitForFlywheels(.03, flywheelTargetRpm);
	indexFeeder(600, 800);
	indexFeeder(600, 0);
	waitForBaseTimeout(15, 1500);
	//back up to mid-field
	baseTargetPositionStraightSet(-60);
	delay(700);
	//fire preloads
	indexFeeder(600, 700);
	indexFeeder(600, 700);
	//we are done
	flywheelTargetRpmSet(0);
	while (1) {
		baseTargetPosition = baseEncAveGet();
		delay(20);
	}
}

//autonomous routine to gather up stacks
void hoarding(int color) {
	//drive across field
	baseTargetPositionStraightSet(118);
	waitForBaseTimeout(15, 7000);
	baseEncReset();
	//turn to line up with stacks
	baseTargetPositionTurnSet(22 * color);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//back into stacks to wall
	baseTargetPositionStraightSet(-90);
	waitForBaseTimeout(15, 5000);
	baseEncReset();
	//pull off of wall
	baseTargetPositionStraightSet(2);
	waitForBaseTimeout(15, 2000);
	baseEncReset();
	//turn to be parallel to wall
	baseTargetPositionTurnSet(-60*color);
	waitForBaseTimeout(15, 3500);
	baseEncReset();
	//drive along wall
	baseTargetPositionStraightSet(-48);
	waitForBaseTimeout(15, 5000);
	baseEncReset();
	//turn towards center of field
	baseTargetPositionTurnSet(41 * color);
	waitForBaseTimeout(15, 3500);
	baseEncReset();
	//drive along stack
	baseTargetPositionStraightSet(48);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//turn to face stack
	baseTargetPositionTurnSet(-30 * color);
	waitForBaseTimeout(15, 3000);
	baseEncReset();
	//back into stack
	baseTargetPositionStraightSet(-60);
	waitForBaseTimeout(15, 2000);
	flywheelTargetRpmSet(1500);
	setConveyor(127);
	baseEncReset();
	//turn to face goal
	baseTargetPositionTurnSet(-155 * color);
	waitForBaseTimeout(15, 4500);
	waitForFlywheels(.02, flywheelTargetRpm);
	//fire 1 preload
	indexFeeder(600, 0);
	flywheelTargetRpmSet(1300);
	baseEncReset();
	//drive to midfield
	baseTargetPositionStraightSet(-36);
	waitForBaseTimeout(15, 4000);
	//fire remaining preloads
	indexFeeder(600, 0);
	waitForFlywheels(.03, flywheelTargetRpm);
	indexFeeder(600, 0);
	waitForFlywheels(.03, flywheelTargetRpm);
	indexFeeder(600, 0);
	waitForFlywheels(.03, flywheelTargetRpm);
	baseEncReset();
	//drive to low goal
	baseTargetPositionStraightSet(-80);
	waitForBaseTimeout(15,4000);
	//we are done
	flywheelTargetRpmSet(0);
	while(1){
		baseEncReset();
		baseTargetPosition = baseEncAveGet();
		delay(20);
	}
}

//autonomous routine to score preloads using velocity controller
void scorePreloadsSmart() {
	//start flywheels
	flywheelTargetRpm = 1650;
	waitForFlywheels(.01, flywheelTargetRpm);
	setConveyor(127);
	delay(500);
	//fire 4 preloads
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

//turn to drive out to get stacks
void repositionForStacks(int color) {
	baseEncReset();
	baseTargetPositionStraightSet(0);
	waitForBase(15);
	baseEncReset();
	//turn towards wall
	baseTargetPositionTurnSet(-135*color);
	waitForBase(15);
	baseEncReset();
	//straighten against wall
	baseTargetPositionStraightSet(-36);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//pull off of wall
	baseTargetPositionStraightSet(9);
	waitForBase(15);
	baseEncReset();
}

// pick up 2 stacks and score them
void pickUpScore2Stacks(int color) {
	baseEncReset();
	//turn towards stacks
	baseTargetPositionTurnSet(-15*color);
	waitForBase(15);
	baseEncReset();
	//start conveyor
	setConveyor(127);
	time = millis();
	//drive over to stacks
	baseTargetPositionStraightSet(60);
	waitForBaseTimeout(15, 5000);
	baseEncReset();
	//turn to face stacks
	baseTargetPositionTurnSet(100*color);
	waitForBaseTimeout(15, 5000);
	baseEncReset();
	//drive into stacks
	baseTargetPositionStraightSet(20);
	//start flywheels
	flywheelTargetRpmSet(1350);
	waitForBaseTimeout(15, 5000);
	//get rid of any that are jammed
	setConveyor(-127);
	delay(800);
	setConveyor(127);
	delay(1300);
	setConveyor(-127);
	delay(900);
	setConveyor(127);
	baseEncReset();
	//turn to face goal
	baseTargetPositionTurnSet(34*color);
	waitForBaseTimeout(15, 4500);
	waitForFlywheels(.02, flywheelTargetRpm);
	//fire balls
	for (int i = 0; i < 3; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		waitForFlywheels(.02, flywheelTargetRpm);
	}
	//turn towards next stack
	baseEncReset();
	baseTargetPositionTurnSet(-29*color);
	waitForBase(15);
	baseEncReset();
	//pick up next stack
	baseTargetPositionStraightSet(24);
	waitForBaseTimeout(15, 3000);
	baseTargetPositionStraightSet(6);
	waitForBaseTimeout(15, 2000);
	//get rid of any that are jammed
	setConveyor(-127);
	delay(1300);
	setConveyor(127);
	delay(200);
	baseEncReset();
	//turn to face goal
	baseTargetPositionTurnSet(28*color);
	waitForBase(15);
	waitForFlywheels(.02, flywheelTargetRpm);
	//fire balls
	for (int i = 0; i < 8; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		waitForFlywheels(.02, flywheelTargetRpm);
	}
	flywheelTargetRpmSet(0);
}

//programming skills routine

void progSkills(int color) {
	time = millis();
	//drive across field
	baseTargetPositionStraightSet(-112);
	waitForBase(15);
	baseEncReset();
	//turn to be orthogonal to wall
	baseTargetPositionTurnSet(-90*color);
	waitForBase(15);
	baseEncReset();
	//start flywheels
	flywheelTargetRpmSet(1350); //1525 for full field
	//straighten against wall
	baseTargetPositionStraightSet(52);
	waitForBaseTimeout(15, 3750);
	baseEncReset();
	//pull off of wall
	baseTargetPositionStraightSet(-18);
	waitForBase(15);
	setConveyor(127);
	baseEncReset();
	// turn to face goal
	baseTargetPositionTurnSet(7*color);
	waitForBase(15);
	waitForFlywheels(.02, flywheelTargetRpm);
	//fire matchloads
	for (int i = 0; i < 38; i++) {
		indexFeeder(700, 0);
		hasCrossedL = 0;
		hasCrossedR = 0;
		while (abs(flywheelTargetRpm - rpmL) > .02 * flywheelTargetRpm
				|| abs(flywheelTargetRpm - rpmR) > .02 * flywheelTargetRpm) {
			if (millis() - time > 50000)
				break;
			delay(20);
		}
		if (millis() - time > 50000)
			break;
	}
	//stop flywheels and conveyors
	flywheelTargetRpmSet(0);
	setConveyor(0);
	baseEncReset();
	baseTargetPositionStraightSet(0);
	waitForBase(15);
	baseEncReset();
	//turn to face 15" robot
	baseTargetPositionTurnSet(84*color);
	waitForBase(15);
	baseEncReset();
	//drive into wall
	baseTargetPositionStraightSet(-24);
	waitForBaseTimeout(15, 1500);
	baseEncReset();
	baseTargetPositionStraightSet(0);
	//deploy ramps
	deployRamps();
}

//autonomous mode
void autonomous() {
	startUpdaterTasks();
	//run the selected routine
	if (autonomousMode == skills)
		progSkills(autonomousColor);
	else if (autonomousMode == stacks) {
		scorePreloadsSmart();
		repositionForStacks(autonomousColor);
		pickUpScore2Stacks(autonomousColor);
		flywheelTargetRpmSet(0);
		setConveyor(0);
	} else if (autonomousMode == preloads)
		scorePreloadsSmart();
	else if (autonomousMode == intercept)
		ram(autonomousColor);
	else if(autonomousMode == hoard)
		hoarding(autonomousColor);

	while (1) {
		//printf("%d, %d;  ", baseEncoderLGet(), baseEncoderRGet());
		delay(20);
	}
}
