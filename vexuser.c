#include <stdlib.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header
#include "robotc_glue.h"

// Digi IO configuration
static vexDigiCfg dConfig[kVexDigital_Num] = { { kVexDigital_1,
		kVexSensorDigitalInput, kVexConfigInput, 0 }, //d1
		{ kVexDigital_2, kVexSensorQuadEncoder, kVexConfigQuadEnc1,
				kVexQuadEncoder_1 }, //d2
		{ kVexDigital_3, kVexSensorQuadEncoder, kVexConfigQuadEnc2,
				kVexQuadEncoder_1 }, //d3
		{ kVexDigital_4, kVexSensorQuadEncoder, kVexConfigQuadEnc1,
				kVexQuadEncoder_2 }, //d4
		{ kVexDigital_5, kVexSensorQuadEncoder, kVexConfigQuadEnc2,
				kVexQuadEncoder_2 }, //d5
		{ kVexDigital_6, kVexSensorQuadEncoder, kVexConfigQuadEnc1,
				kVexQuadEncoder_3 }, //d6
		{ kVexDigital_7, kVexSensorQuadEncoder, kVexConfigQuadEnc2,
				kVexQuadEncoder_3 }, //d7
		{ kVexDigital_8, kVexSensorDigitalInput, kVexConfigInput, 0 }, //d8
		{ kVexDigital_9, kVexSensorDigitalInput, kVexConfigInput, 0 }, //d9
		{ kVexDigital_10, kVexSensorDigitalInput, kVexConfigInput, 0 }, //d10
		{ kVexDigital_11, kVexSensorDigitalInput, kVexConfigInput, 0 }, //d11
		{ kVexDigital_12, kVexSensorDigitalOutput, kVexConfigOutput, 0 } }; //d12

//motor config
static vexMotorCfg mConfig[kVexMotorNum] =
		{ { kVexMotor_1, kVexMotor393T, kVexMotorNormal, kVexSensorIME,
				kImeChannel_1 }, { kVexMotor_2, kVexMotorUndefined,
				kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_3,
				kVexMotorUndefined, kVexMotorReversed, kVexSensorNone, 0 }, {
				kVexMotor_4, kVexMotorUndefined, kVexMotorNormal,
				kVexSensorNone, 0 }, { kVexMotor_5, kVexMotorUndefined,
				kVexMotorNormal, kVexSensorNone, 0 }, { kVexMotor_6,
				kVexMotorUndefined, kVexMotorNormal, kVexSensorNone, 0 }, {
				kVexMotor_7, kVexMotorUndefined, kVexMotorNormal,
				kVexSensorNone, 0 }, { kVexMotor_8, kVexMotorUndefined,
				kVexMotorReversed, kVexSensorNone, 0 }, { kVexMotor_9,
				kVexMotorUndefined, kVexMotorReversed, kVexSensorNone, 0 }, {
				kVexMotor_10, kVexMotor393T, kVexMotorNormal, kVexSensorIME,
				kImeChannel_2 } };

/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void vexUserSetup() {
	vexDigitalConfigure(dConfig, DIG_CONFIG_SIZE( dConfig ));
	vexMotorConfigure(mConfig, MOT_CONFIG_SIZE( mConfig ));
}

//disabled/enabled by field control
int16_t isEnabled(void) {
	return (vexControllerCompetitonState() & kFlagDisabled ? 0 : 1);
}

//auton routines
#define	red	1
#define	blue	-1
#define stacks	1
#define intercept 2
#define preloads 3
#define hoard 4
#define interceptSimple 5
#define skills 6
#define nothing 7

//default mode & color
int autonomousMode = preloads;
int autonomousColor = red;
//LCD menu to select routine
void lcdAutoSelection(void) {
	short pageAuto = 0, maxPageAuto = 7;
	short autoSelected = 0;
	short pageColor = red;
	while (!isEnabled()) {
		//have not selected routine yet
		if (!autoSelected) {
			//display one routine at a time
			if (pageAuto == stacks)
				vexLcdSet(0, 0, "stacks");
			else if (pageAuto == intercept)
				vexLcdSet(0, 0, "intercept");
			else if (pageAuto == preloads)
				vexLcdSet(0, 0, "preloads only");
			else if (pageAuto == hoard)
				vexLcdSet(0, 0, "hoarding");
			else if (pageAuto == skills)
				vexLcdSet(0, 0, "skills");
			else if (pageAuto == nothing)
				vexLcdSet(0, 0, "do nothing");
			else if (pageAuto == interceptSimple)
				vexLcdSet(0, 0, "simple intercept");

			//loop around
			if (pageAuto < 1)
				pageAuto = maxPageAuto;
			else if (pageAuto > maxPageAuto)
				pageAuto = 1;

			//move between routines L/R
			if (vexLcdButtonGet(0) == kLcdButtonLeft) {
				pageAuto--;
				while (vexLcdButtonGet(0) == kLcdButtonLeft)
					vexSleep(25);
			} else if (vexLcdButtonGet(0) == kLcdButtonRight) {
				pageAuto++;
				while (vexLcdButtonGet(0) == kLcdButtonRight)
					vexSleep(25);
			}

			//select a routine Center
			if (vexLcdButtonGet(0) == kLcdButtonCenter) {
				autonomousMode = pageAuto;
				if (pageAuto == stacks)
					vexLcdSet(0, 1, "stacks");
				else if (pageAuto == intercept)
					vexLcdSet(0, 1, "intercept");
				else if (pageAuto == preloads)
					vexLcdSet(0, 1, "preloads only");
				else if (pageAuto == hoard)
					vexLcdSet(0, 1, "hoarding");
				else if (pageAuto == skills)
					vexLcdSet(0, 1, "skills");
				else if (pageAuto == nothing)
					vexLcdSet(0, 1, "do nothing");
				else if (pageAuto == interceptSimple)
					vexLcdSet(0, 1, "simple intercept");
				autoSelected = 1;
			}
			//select color
		} else {
			if (pageColor == red)
				vexLcdSet(0, 0, "red");
			else
				vexLcdSet(0, 0, "blue");

			//change color
			if (vexLcdButtonGet(0) == kLcdButtonRight) {
				pageColor = -1 * pageColor;
				while (vexLcdButtonGet(0) == kLcdButtonRight)
					vexSleep(25);
			}

			//go back
			if (vexLcdButtonGet(0) == kLcdButtonLeft)
				autoSelected = 0;

			//save selction
			autonomousColor = pageColor;
		}
		vexSleep(25);
	}
}

/*-----------------------------------------------------------------------------*/
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void vexUserInit() {
	lcdAutoSelection();
}

//encoder conversions for base in auton
#define ticksPerIn	28.10 //25.87 //26.23 //27.43 // 27.64
#define ticksPerDeg	3.2 //471 163 // 392 253
//retrieve sign of a variable
short sign(int var) {
	if (var > 0)
		return 1;
	else if (var < 0)
		return -1;
	else
		return 0;
}

//convert analog reading to 1 or 0 for use with buttons/limit switches
//returns 1 not pressed, 0 pressed
short analogToDigital(short port) {
	if (vexAdcGet(port) > 100)
		return 1;
	else
		return 0;
}

//old
//const unsigned int trueFlywheelSpeed[128] = { 0, 0, 13, 13, 14, 14, 14, 15, 15,
//		15, 15, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19,
//		19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 24, 25,
//		25, 25, 26, 26, 26, 27, 27, 27, 28, 28, 29, 30, 31, 31, 32, 32, 33, 33,
//		34, 34, 34, 35, 35, 36, 37, 38, 38, 39, 39, 40, 41, 42, 43, 43, 44, 45,
//		46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 63, 64, 65,
//		66, 69, 70, 72, 73, 74, 75, 75, 76, 77, 78, 78, 79, 79, 79, 80, 80, 81,
//		82, 83, 84, 85, 86, 87, 88, 89, 92, 105, 127 };

//new
const unsigned int trueFlywheelSpeed[128] = { 0, 14, 15, 16, 16, 16, 17, 17, 17,
		18, 18, 18, 18, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 23, 23,
		23, 23, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 27, 27, 28, 28, 28, 29,
		29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37,
		38, 38, 39, 39, 40, 40, 41, 42, 43, 44, 44, 45, 46, 47, 47, 48, 48, 49,
		50, 51, 52, 53, 54, 56, 56, 57, 57, 59, 60, 61, 62, 62, 63, 64, 66, 67,
		68, 68, 69, 71, 72, 73, 74, 75, 75, 76, 77, 78, 79, 79, 80, 80, 81, 82,
		82, 83, 84, 85, 85, 85, 86, 90, 103, 110, 127 };

//set flywheel motors to command
void setFlywheel(short command) {
	vexMotorSet(kVexMotor_4, trueFlywheelSpeed[command]);
	vexMotorSet(kVexMotor_5, trueFlywheelSpeed[command]);
	vexMotorSet(kVexMotor_6, trueFlywheelSpeed[command]);
	vexMotorSet(kVexMotor_7, trueFlywheelSpeed[command]);
}
//get current control value of flywheel motors
short getFlywheel(void) {
	return vexMotorGet(kVexMotor_4);
}
//accelerate flywheel to target speed
void accelerateFlywheel(short target) {
	short command = getFlywheel();
	while (command < target) {
		command++;
		setFlywheel(command);
		vexSleep(50);
	}
	while (command > target) {
		command--;
		setFlywheel(command);
		vexSleep(50);
	}
}

double getFlywheelVelocity(void) {
	static double encoderCurrent, encoderLast = 0, tLast = 0;
	double encoderDelta, dt;
	//update current position
	encoderCurrent = vexEncoderGet(kVexQuadEncoder_1);
	//derive change in encoder position
	encoderDelta = encoderCurrent - encoderLast;
	//derive change in time
	dt = chTimeNow() - tLast;
	//store position
	encoderLast = encoderCurrent;
	//stor time
	tLast = chTimeNow();

	//calculate rotational velocity in rpm
	return encoderDelta * (2.0 * -35.0 / 3.0) / 360.0 * (60.0 * 1000.0 / dt);
}

double getDriveVelocity(void) {
	static double encoderCurrent, encoderLast = 0, tLast = 0;
	double encoderDelta, dt;
	//update current position
	encoderCurrent = vexEncoderGet(kVexQuadEncoder_2);
	//derive change in encoder position
	encoderDelta = encoderCurrent - encoderLast;
	//derive change in time
	dt = chTimeNow() - tLast;
	//store position
	encoderLast = encoderCurrent;
	//stor time
	tLast = chTimeNow();

	//calculate rotational velocity in rpm
	return encoderDelta / 360.0 * (60.0 * 1000.0 / dt);
}

//task to update flywheel v
double flywheelVelocity = 0.0;
double driveVelocity = 0.0;
task calcVelocity(void *arg) {
	(void) arg;

	vexTaskRegister("calcV");

	while (TRUE) {
		//IIR filter 25% current v
		flywheelVelocity = flywheelVelocity * 0.75
				+ 0.25 * getFlywheelVelocity();
		//driveVelocity = getDriveVelocity();
		wait1Msec(25);
	}
}

long batteryVoltage;
//flywheel velocity controller
static char hasCrossed = 0;
float tbhController(long target) {
	//error and previous error in v
	static long error, pError = 0;

	//proportionality constant
	const float kP = .015;
	//flywheel motor setting
	static float speed = 0, speedAtZero = 0;

	//calculate error
	error = target - flywheelVelocity;
	//speed is integral of error times scale factor
	speed += error * kP;

	//set motors to full power after firing flywheel
	if (error > 200 && flywheelVelocity > 1000) {
		hasCrossed = 0;
		speed = 127;
	}

	//cap speed at full power
	if (speed > 127)
		speed = 127;
	//don't go backwards
	else if (speed < 0)
		speed = 0;

	//take back half: adjust speed when error crosses 0
	if (error * pError <= 0) {
		if (!hasCrossed) {
			speed = 127.0 * target / 3400 * 7600.0 / vexSpiGetMainBattery();
			hasCrossed = 1;
		} else
			speed = .5 * (speed + speedAtZero);
		speedAtZero = speed;
	}
	pError = error;

	//set LED to indicate flywheels at correct velocity
	if (abs(error) <= .02 * target)
		vexDigitalPinSet(kVexDigital_12, 0);
	else
		vexDigitalPinSet(kVexDigital_12, 1);

	return speed;
}

short comand, i;

int fwTarget = 0;

//task to run in parallel to update velocity controller
task vControlUpdate(void *arg) {
	(void) arg;

	vexTaskRegister("vControl");

	int isHeld = 0;
	batteryVoltage = vexSpiGetMainBattery();

	vex_printf("t, v, c, s, f															            ");

	while (TRUE) {
		//starting tile preset
		if (vexControllerGet(Btn8U))
			fwTarget = 2500;
		//off
		else if (vexControllerGet(Btn8D))
			fwTarget = 0;
		//back corner preset
		else if (vexControllerGet(Btn7U))
			fwTarget = 2700;
		//midfield preset
		else if (vexControllerGet(Btn7R))
			fwTarget = 2025;
		//bar preset
		else if (vexControllerGet(Btn7L))
			fwTarget = 1900;
		//skills
		else if (vexControllerGet(Btn7D))
			fwTarget = 2050;
		//manual increase
		else if (vexControllerGet(Btn8R)) {
			if (!isHeld) {
				fwTarget += 25;
				isHeld = 1;
			}
			//manual decrease
		} else if (vexControllerGet(Btn8L)) {
			if (!isHeld) {
				fwTarget -= 25;
				isHeld = 1;
			}
		} else
			isHeld = 0;

		//manually set to max
		if (vexControllerGet(Btn6U))
			setFlywheel(127);
		//else use v controller
		else
			setFlywheel(tbhController(fwTarget));
//print values to terminal
		vex_printf("%d, %f, %d,	%d													            ", chTimeNow(),
				flywheelVelocity, 10 * getFlywheel(),
				vexSpiGetMainBattery() / 10);

		vexSleep(25);
	}
}

//set the feeder motor to the value of command
void setFeeder(short command) {
	vexMotorSet(0, command);
}
//get the value of the feeder sensor
//returns 1 no ball, 0 ball in place
short getFeederState(void) {
	return vexDigitalPinGet(0);
}
//indexes the feeder to advance 1 ball; ie fire 1 ball
void indexFeeder(void) {
	static short lastState;
	setFeeder(-127);
	//wait for limit switch release
	while (getFeederState() == 0 || lastState != getFeederState()) {
		lastState = getFeederState();
		vexSleep(25);
	}
	//wait for limit switch pressed again
	//hasCrossed = FALSE;
	while (getFeederState() == 1 || lastState != getFeederState()) {
		lastState = getFeederState();
		vexSleep(25);
	}
	setFeeder(0);
}

//index feeder but stop after certain time if no balls pass through
void indexFeederTimeout(long time) {
	static short lastState;
	setFeeder(-127);
	while (getFeederState() == 0 || lastState != getFeederState()) {
		lastState = getFeederState();
		vexSleep(25);
	}
	hasCrossed = FALSE;
	while (getFeederState() == 1 || lastState != getFeederState()) {
		lastState = getFeederState();
		if (time <= 0) {
			setFeeder(0);
			return;
		}
		time -= 25;
		vexSleep(25);
	}
	setFeeder(0);
}

//wait for flywheels to recover to target rpm
void waitForFlywheel(float percentError, long target) {
	while (abs(target - flywheelVelocity) > percentError * target) {
		vexSleep(50);
	}
}

const unsigned int linearDrive[128] = { 0, 2, 4, 6, 8, 10, 12, 14, 15, 16, 17,
		18, 18, 18, 19, 19, 19, 20, 20, 20, 20, 20, 21, 22, 22, 23, 23, 23, 23,
		23, 24, 24, 25, 25, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 30,
		31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38,
		38, 39, 39, 40, 40, 41, 41, 42, 42, 43, 43, 44, 45, 46, 47, 47, 48, 50,
		50, 51, 53, 54, 55, 58, 59, 59, 60, 60, 61, 62, 63, 65, 67, 68, 69, 72,
		73, 75, 77, 79, 80, 81, 81, 82, 83, 84, 85, 127, 127, 127, 127, 127,
		127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127 };

//set drive motors
void driveSet(short left, short right) {
	vexMotorSet(kVexMotor_2, sign(left) * linearDrive[abs(left)]);
	vexMotorSet(kVexMotor_3, sign(left) * linearDrive[abs(left)]);
	vexMotorSet(kVexMotor_8, sign(right) * linearDrive[abs(right)]);
	vexMotorSet(kVexMotor_9, sign(right) * linearDrive[abs(right)]);
}

//retreive position of left base encoder
long baseEncoderLGet(void) {
	return vexEncoderGet(kVexQuadEncoder_2);
} // retreive position of right base encoder
long baseEncoderRGet(void) {
	return vexEncoderGet(kVexQuadEncoder_3);
} //retreive average of left and right base encoders
long baseEncAveGet(void) {
	return .5 * (baseEncoderLGet() + baseEncoderRGet());
} //same as above, but used for turning
long absBaseEncAveGet(void) {
	return .5 * (abs(baseEncoderLGet()) + abs(baseEncoderRGet()))
			* sign(baseEncoderRGet());
} //set base encoders to 0
void baseEncReset(void) {
	vexEncoderSet(kVexQuadEncoder_2, 0);
	vexEncoderSet(kVexQuadEncoder_3, 0);
}

#define straight 0
#define turn 1
short dir, dumb;
long baseTargetPosition;
short baseCommandL, baseCommandR;
short pControl = 1;
//used for driving straight in autonomous
void baseTargetPositionStraightSet(long inches) {
	baseTargetPosition = ticksPerIn * inches;
	dir = straight;
} //used for turning in autonomous
void baseTargetPositionTurnSet(long degrees) {
	baseTargetPosition = ticksPerDeg * degrees;
	dir = turn;
} //just set base motor control value
void baseSimpleSet(short left, short right) {
	pControl = 0;
	driveSet(left, right);
}

//wait for base to reach target position and stabilize
void waitForBase(float margin) {
	static long pBaseEncAve[5] = { 0, 0, 0, 0, 0 };
	if (dir == turn) {
		//wait 125ms to make sure we are stopped
		while (abs(baseTargetPosition - absBaseEncAveGet()) > margin
				|| absBaseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = absBaseEncAveGet();
			vexSleep(25);
		}
	} else {
		while (abs(baseTargetPosition - baseEncAveGet()) > margin
				|| baseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = baseEncAveGet();
			vexSleep(25);
		}
	}
}

//same as above, but quit waiting after certain time if something went wrong
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
			timeout -= 25;
			if (timeout < 0)
				return;
			vexSleep(25);
		}
	} else {
		while (abs(baseTargetPosition - baseEncAveGet()) > margin
				|| baseEncAveGet() != pBaseEncAve[4]) {
			pBaseEncAve[4] = pBaseEncAve[3];
			pBaseEncAve[3] = pBaseEncAve[2];
			pBaseEncAve[2] = pBaseEncAve[1];
			pBaseEncAve[1] = pBaseEncAve[0];
			pBaseEncAve[0] = baseEncAveGet();
			timeout -= 25;
			if (timeout < 0)
				return;
			vexSleep(25);
		}
	}
}

//position controller for base in auto
task pControllerUpdate(void *arg) {
	(void) arg;
	vexTaskRegister("pControllerUpdate");

	long errorDist, errorDiff, distanceProportion, differenceProportion;
	long distanceProportionMax = 110;
	float kPDist = .044, kPDiff = .6;
	float minCommand, minCommandSt = 10;
	short brakingPower = -1, kAccel = 3;
	static short pBaseCommandL = 0, pBaseCommandR = 0;
	static long pBaseEncoderL = 0, pBaseEncoderR = 0;
	static long pBaseTargetPosition;

	float kPTurn = .24;
	float minCommandTurn, minCommandTurnSt = 17;

	minCommand = minCommandSt;
	minCommandTurn = minCommandTurnSt;

	while (1) {

		//reset minCommands when given new target
		if (baseTargetPosition != pBaseTargetPosition) {
			minCommandTurn = minCommandTurnSt;
			minCommand = minCommandSt;
		}

		//turning
		if (dir == turn) {

			//calulate error
			errorDist = baseTargetPosition - absBaseEncAveGet();
			//multiply by proportionality constant
			distanceProportion = kPTurn * errorDist;

			//cap proportion
			if (abs(distanceProportion)
					> distanceProportionMax - minCommandTurn)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommandTurn);

			//calculate difference between sides
			errorDiff = baseEncoderLGet() + baseEncoderRGet();
			differenceProportion = kPDiff * errorDiff;

			//set motors to reflect distance to target & difference between sides plus minimum
			baseCommandL = minCommandTurn * -1 * sign(errorDist)
					- distanceProportion - differenceProportion;
			baseCommandR = minCommandTurn * sign(errorDist) + distanceProportion
					- differenceProportion;

			//aply breaking power when we are close
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

			//increase minimum command if we stall
			if ((abs(baseCommandL) >= minCommandTurn
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommandTurn
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommandTurn += .1;

			//store previous values
			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();

			//else we must be going straight
		} else {

			//calculate disatnce to target
			errorDist = baseTargetPosition - baseEncAveGet();
			distanceProportion = kPDist * errorDist;

			//cap proportion
			if (abs(distanceProportion) > distanceProportionMax - minCommand)
				distanceProportion = sign(distanceProportion)
						* (distanceProportionMax - minCommand);

			//calculate difference between sides
			errorDiff = baseEncoderLGet() - baseEncoderRGet();
			differenceProportion = kPDiff * errorDiff;

			//distance, difference, and minimum power
			baseCommandL = minCommand * sign(errorDist) + distanceProportion
					- differenceProportion;
			baseCommandR = minCommand * sign(errorDist) + distanceProportion
					+ differenceProportion;

			//apply braking power when we are close
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
			if ((abs(baseCommandL) >= minCommand
					&& pBaseEncoderL == baseEncoderLGet())
					|| (abs(baseCommandR) >= minCommand
							&& pBaseEncoderR == baseEncoderRGet()))
				minCommand += .1;

			//store previous values
			pBaseCommandL = baseCommandL;
			pBaseCommandR = baseCommandR;

			pBaseEncoderL = baseEncoderLGet();
			pBaseEncoderR = baseEncoderRGet();
		}
		//set motors
		if (pControl != 0)
			driveSet(baseCommandL, baseCommandR);
		//print values
		vex_printf("%d, %d;  ", errorDist, errorDiff);
		pBaseTargetPosition = baseTargetPosition;

		vexSleep(25);
	}
}

void driveTesting(void) {
	baseTargetPositionTurnSet(90);
	while (1)
		vexSleep(25);
}

//auto routine to fire preloads - does not use v controller
void firePreloadsDumb(void) {
	accelerateFlywheel(85);
	vexSleep(1000);
	indexFeeder();
	vexSleep(2000);
	indexFeeder();
	vexSleep(2000);
	setFeeder(-100);
	vexSleep(3000);
	setFeeder(0);
	accelerateFlywheel(0);
}

long time;

//auton routine to get oponents' stacks
void oppositeSide(int color) {
	fwTarget = 2225;
	baseEncReset();
	//drive up
	baseTargetPositionStraightSet(36);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//turn to goal
	baseTargetPositionTurnSet(-35 * color);
	waitForBaseTimeout(15, 4000);
	//fire a couple preloads
	indexFeeder();
	vexSleep(400);
	indexFeeder();
	baseEncReset();
	//drive over
	baseTargetPositionStraightSet(32);
	fwTarget = 2125;
	waitForBaseTimeout(15, 4000);
	//fire remaining preloads
	indexFeederTimeout(1000);
	indexFeederTimeout(1000);
	baseEncReset();
	//turn to wall
	baseTargetPositionTurnSet(40 * color);
	fwTarget = 2125;
	waitForBaseTimeout(15, 3000);
	baseEncReset();
	//drive to wall
	baseTargetPositionStraightSet(52);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//turn to goal
	baseTargetPositionTurnSet(-52 * color);
	waitForBaseTimeout(15, 4000);
	//fire any balls we may have gotten
	indexFeederTimeout(1000);
	vexSleep(1000);
	indexFeederTimeout(1000);
	indexFeederTimeout(600);
	baseEncReset();
	fwTarget = 0;
	//we are done
	while (1) {
		baseEncReset();
		baseTargetPosition = baseEncAveGet();
		vexSleep(25);
	}
}

//simply drive diagonal to wall across oponents and fire
void ram(int color) {
	fwTarget = 2050;
	baseEncReset();
	//drive forward to wall
	baseTargetPositionStraightSet(-114);
	waitForBaseTimeout(15, 6000);
	fwTarget = 2050;
	baseEncReset();
	//turn to goal
	baseTargetPositionTurnSet((-55 - 180) * color);
	waitForBaseTimeout(15, 4000);
	//fire preloads
	indexFeederTimeout(900);
	indexFeederTimeout(900);
	indexFeederTimeout(900);
	fwTarget = 0;
	//we are done
	while (1) {
		baseEncReset();
		baseTargetPosition = baseEncAveGet();
		vexSleep(25);
	}
}

//gather up stacks near us
void hoarding(int color) {
	time = chTimeNow();
	baseEncReset();
	//drive to nearest stacks
	baseTargetPositionStraightSet(50);
	waitForBaseTimeout(15, 5000);
	baseEncReset();
	//turn so back is facing them
	baseTargetPositionTurnSet(-40 * color);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//back up to wall, pushing stacks
	baseTargetPositionStraightSet(-40);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//pull off of wall
	baseTargetPositionStraightSet(2);
	waitForBaseTimeout(15, 3000);
	baseEncReset();
	//turn to be parallel with wall
	baseTargetPositionTurnSet(90 * color);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//back up into loading zone
	baseTargetPositionStraightSet(-42);
	waitForBaseTimeout(15, 7000);
	//if nothing went horribly wrong . . .
	if ((chTimeNow() - time) < 20000) {
		vexSleep(6000);
		baseEncReset();
		//turn to face goal
		baseTargetPositionTurnSet(-45 * color);
		waitForBaseTimeout(15, 2500);
		fwTarget = 2125;
		baseEncReset();
		//drive to mid-field
		baseTargetPositionStraightSet(90);
		waitForBaseTimeout(15, 5000);
		//fire preloads
		indexFeeder();
		indexFeeder();
		indexFeederTimeout(1000);
		fwTarget = 0;
		baseEncReset();
		//back up behind stacks
		baseTargetPositionStraightSet(-24);
		waitForBaseTimeout(15, 2500);
		baseEncReset();
		//turn perpendicular to wall
		baseTargetPositionTurnSet(-45 * color);
		waitForBaseTimeout(15, 3000);
		baseEncReset();
		//drive to wall
		baseTargetPositionStraightSet(72);
		waitForBaseTimeout(15, 4500);
		baseEncReset();
		//back off of wall
		baseTargetPositionStraightSet(-2);
		waitForBaseTimeout(15, 1500);
		baseEncReset();
		//turn to be parallel with wall
		baseTargetPositionTurnSet(-90 * color);
		waitForBaseTimeout(15, 4000);
		baseEncReset();
		//back up along wall to low goal
		baseTargetPositionStraightSet(-48);
		waitForBaseTimeout(15, 3500);
		baseEncReset();
	}
	while (1) {
		baseEncReset();
		baseTargetPosition = baseEncAveGet();
		vexSleep(25);
	}

}

//fire preloads using velocity controller
void firePreloadsSmart(void) {
	fwTarget = 2500;
	waitForFlywheel(.02, fwTarget);
	for (i = 0; i < 3; i++) {
		waitForFlywheel(.01, fwTarget);
		vexSleep(100);
		waitForFlywheel(.01, fwTarget);
		indexFeederTimeout(3000);
	}
	vexSleep(500);
	fwTarget = 0;
}

//turn to drive to stack after firing preloads
void repositionForStack(int color) {
	baseEncReset();
//	baseTargetPositionStraightSet(10);
//	waitForBase(15);
//	baseEncReset();
//	baseTargetPositionTurnSet(-39);
//	waitForBase(15);
//	baseEncReset();
//	baseTargetPositionStraightSet(-30);
//	waitForBaseTimeout(15, 1200);
//	baseEncReset();
//	baseTargetPositionStraightSet(9);
//	waitForBase(15);
	baseTargetPositionTurnSet(-18 * color);
	waitForBaseTimeout(15, 3000);
	baseEncReset();
}

//intake stack off wall
void pickUpStack(int color) {
	time = chTimeNow();
	fwTarget = 2100;
	baseEncReset();
	//drive to mid-field
	baseTargetPositionStraightSet(42);
	waitForBaseTimeout(15, 10000);
	baseEncReset();
	//turn to face wall
	baseTargetPositionTurnSet(-120 * color);
	waitForBaseTimeout(15, 10000);
	if (chTimeNow() - time > 18000) {
		while (1) {
			baseEncReset();
			baseTargetPosition = baseEncAveGet();
			vexSleep(25);
		}
	}
	baseEncReset();
	//drive to wall
	baseTargetPositionStraightSet(48);
	indexFeederTimeout(1500);
	waitForBaseTimeout(15, 4000);
	baseEncReset();
	//turn against wall to pick up stack
	baseTargetPositionTurnSet(-40 * color);
	waitForBaseTimeout(15, 1000);
	//baseEncReset();
	baseTargetPositionTurnSet(40 * color);
	waitForBaseTimeout(15, 2000);
	//baseEncReset();
	baseTargetPositionTurnSet(0 * color);
	waitForBaseTimeout(15, 1000);
	//baseEncReset();
	baseTargetPositionStraightSet(4);
	waitForBaseTimeout(15, 750);
	//baseEncReset();
	baseTargetPositionStraightSet(-0);
	waitForBase(15);
	//baseEncReset();
//	baseTargetPositionStraightSet(4);
//	waitForBaseTimeout(15, 4000);
	//baseEncReset();
//	baseTargetPositionStraightSet(0);
//	waitForBase(15);
	baseTargetPositionTurnSet(0 * color);
	waitForBase(15);
	//back off of wall
	baseTargetPositionStraightSet(-35);
	waitForBase(15);
}

//turn to face goal and shoot now that we've gotten stack
void turnAndShoot(int color) {
	fwTarget = 2175;
	baseEncReset();
	//turn to face goal
	baseTargetPositionTurnSet(137 * color);
	waitForBase(15);
	//fire balls
	for (i = 0; i < 4; i++) {
		waitForFlywheel(.01, fwTarget);
		indexFeederTimeout(750);
	}
}

//programming skills routine
void progSkills(int color) {
	int i;
	time = chTimeNow();
	baseEncReset();
	//drive over 1 tile
	baseTargetPositionStraightSet(20);
	waitForBase(15);
	baseEncReset();
	//turn to be orthogonal to wall
	baseTargetPositionTurnSet(90 * color);
	waitForBase(15);
	baseEncReset();
	//back into wall
	baseTargetPositionStraightSet(-36);
	fwTarget = 2075;		//2275 good
	waitForBaseTimeout(15, 900);
	baseEncReset();
	// drive off of wall
	baseTargetPositionStraightSet(30);
	waitForBase(15);
	baseEncReset();
	//turn to face goal
	baseTargetPositionTurnSet(-4 * color);
	waitForBase(15);
	//fire match loads
	for (i = 0; i < 25; i++) {
		while (abs(fwTarget - flywheelVelocity) > .01 * fwTarget) {
			if (chTimeNow() - time > 44000)
				break;
			vexSleep(50);
		}
		static short lastState;
		setFeeder(-127);
		//wait for limit switch release
		while (getFeederState() == 0 || lastState != getFeederState()) {
			lastState = getFeederState();
			if (chTimeNow() - time > 44000)
							break;
			vexSleep(25);
		}
		//wait for limit switch pressed again
		//hasCrossed = FALSE;
		while (getFeederState() == 1 || lastState != getFeederState()) {
			lastState = getFeederState();
			if (chTimeNow() - time > 44000)
							break;
			vexSleep(25);
		}
		setFeeder(0);
		if (chTimeNow() - time > 44000)
			break;
	}
	fwTarget = 0;
	baseEncReset();
	//drive up to be in line with 24" robot
	baseTargetPositionStraightSet(18);
	waitForBase(15);
	baseEncReset();
	//turn to be facing 24" robot
	baseTargetPositionTurnSet(-88 * color);
	waitForBase(15);
	baseEncReset();
	//straighten against wall
	//baseTargetPositionStraightSet(36);
	//waitForBaseTimeout(15, 1100);
	//baseEncReset();
	//back up to large robot
	baseTargetPositionStraightSet(-18);
	vexSleep(500);
	baseTargetPositionStraightSet(-103);
	waitForBaseTimeout(15, 4750);
	baseEncReset();
	//climb ramps
	baseSimpleSet(-127, -127);
	vexSleep(4500);
	//we are done
	driveSet(-10, -10);
	while (1)
		vexSleep(25);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */
msg_t vexAutonomous(void *arg) {
	(void) arg;

	// Must call this
	vexTaskRegister("auton");

	StartTask(pControllerUpdate);

	StartTask(vControlUpdate);

	StartTask(calcVelocity);

	//run the selected autonomous routine
	if (autonomousMode == skills)
		progSkills(autonomousColor);
	else if (autonomousMode == stacks) {
		firePreloadsSmart();
		repositionForStack(autonomousColor);
		pickUpStack(autonomousColor);
		turnAndShoot(autonomousColor);
	} else if (autonomousMode == preloads)
		firePreloadsSmart();
	else if (autonomousMode == intercept)
		oppositeSide(autonomousColor);
	else if (autonomousMode == interceptSimple)
		ram(autonomousColor);
	else if (autonomousMode == hoard)
		hoarding(autonomousColor);

	while (1) {
		// Don't hog cpu
		vexSleep(25);
	}

	return (msg_t) 0;
}

//task to control feed motor
task backRoller(void *arg) {
	(void) arg;

	vexTaskRegister("backRoller");

	while (TRUE) {
		//index feeder
		if (vexControllerGet(Btn5U)) {
			vexMotorSet(0, -127);
			while (vexDigitalPinGet(0) == 0)
				vexSleep(25);
			//hasCrossed = FALSE;
			while (vexDigitalPinGet(0) == 1)
				vexSleep(25);
			vexMotorSet(0, 0);
		}	//run feeder manually
		else if (vexControllerGet(Btn6D)) {
			while (abs(fwTarget - flywheelVelocity) > .01 * fwTarget) {
				vexSleep(50);
			}
			indexFeeder();

		} else
			vexMotorSet(0, 0);

		wait1Msec(25);
	}
}

const unsigned int trueDriveSpeed[128] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 18, 19, 19, 19, 20, 20, 20, 20, 20, 21, 22, 22, 23, 23, 23, 23, 23,
		24, 24, 25, 25, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 30, 31,
		31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38,
		39, 39, 40, 40, 41, 41, 42, 42, 43, 43, 44, 45, 46, 47, 47, 48, 50, 50,
		51, 53, 54, 55, 58, 59, 59, 60, 60, 61, 62, 63, 65, 67, 68, 69, 72, 73,
		75, 77, 79, 80, 81, 81, 82, 83, 84, 85, 127, 127, 127, 127, 127, 127,
		127, 127, 127, 127, 127, 127, 127, 127, 127, 127 };

//adjust joystick values with deadband
int modifiedDriveControl(int channel) {
	int modifiedCommand, command;
	command = vexControllerGet(channel);
	modifiedCommand = abs(command);
	return sign(command) * trueDriveSpeed[modifiedCommand];
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */
msg_t vexOperator(void *arg) {

	(void) arg;

	// Must call this
	vexTaskRegister("operator");

//	StartTask(pControllerUpdate);

	//driveTesting();

	StartTask(vControlUpdate);

	StartTask(calcVelocity);

	//skills();

//	firePreloadsSmart();
//	repositionForStack();
//	pickUpStack();
//	turnAndShoot();
//	while (1)
//		vexSleep(25);

	StartTask(backRoller);

	//StartTask(control);

	// Run until asked to terminate
	while (!chThdShouldTerminate()) {

//		fwTarget=2150;
//		setFeeder(-127);

//		vex_printf("%f, %d  			  	 			 	 							  			", flywheelVelocity,
//	 vexMotorGet(1));

//		waitForFlywheel(.02, fwTarget);
//		indexFeeder();
		//accelerateFlywheel(0);
//		for (i = 0; i < 128; i++) {
//			setFlywheel(i);
//			vexSleep(2000);
//			vex_printf("%d, %4.2f   												     	 		 	  ", i,
//					flywheelVelocity*7.8/(vexSpiGetMainBattery()/ 10));
//			vexSleep(250);
//			vex_printf("%d, %4.2f   												     	 		 	  ", i,
//					flywheelVelocity*7.8/(vexSpiGetMainBattery() / 10));
//			vexSleep(50);
//		}

		//set base motors
		vexMotorSet(kVexMotor_2, modifiedDriveControl(Ch3));
		vexMotorSet(kVexMotor_3, modifiedDriveControl(Ch3));
		vexMotorSet(kVexMotor_8, modifiedDriveControl(Ch2));
		vexMotorSet(kVexMotor_9, modifiedDriveControl(Ch2));

		// Don't hog cpu
		vexSleep(25);
	}

	return (msg_t) 0;
}
