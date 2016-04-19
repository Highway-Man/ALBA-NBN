/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
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
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {

	//initialize LED's port
	digitalWrite(5, LOW);
	pinMode(5, OUTPUT);

	//initialize pneumatic cylinders' ports
	digitalWrite(1, LOW);
	pinMode(1, OUTPUT);
	digitalWrite(2, LOW);
	pinMode(2, OUTPUT);

}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */

//default autonomous settings
autonomousMode = preloads;
autonomousColor = red;
//lcd autonomous selection menu
void lcdAutoSelection() {
	short pageAuto = 0, maxPageAuto = 6;
	short autoSelected = 0, colorSelected = 0;
	short pageColor = red;
	//while we are disabled before match
	while (isEnabled() == false) {
		//have not selected routine yet
		if (!autoSelected) {
			//display 1 routine option at a time
			if (pageAuto == stacks)
				lcdSetText(uart1, 1, "stacks");
			else if (pageAuto == intercept)
				lcdSetText(uart1, 1, "intercept");
			else if (pageAuto == preloads)
				lcdSetText(uart1, 1, "preloads only");
			else if (pageAuto == hoard)
				lcdSetText(uart1, 1, "hoarding");
			else if (pageAuto == skills)
				lcdSetText(uart1, 1, "skills");
			else if (pageAuto == nothing)
				lcdSetText(uart1, 1, "do nothing");

			//wrap back around when we get to last routine
			if (pageAuto < 1)
				pageAuto = maxPageAuto;
			else if (pageAuto > maxPageAuto)
				pageAuto = 1;

			//switch between routines
			if (lcdReadButtons(uart1) == LCD_BTN_LEFT) {
				pageAuto--;
				while (lcdReadButtons(uart1) == LCD_BTN_LEFT)
					delay(20);
			} else if (lcdReadButtons(uart1) == LCD_BTN_RIGHT) {
				pageAuto++;
				while (lcdReadButtons(uart1) == LCD_BTN_RIGHT)
					delay(20);
			}

			//select current routine with center button
			if (lcdReadButtons(uart1) == LCD_BTN_CENTER) {
				autonomousMode = pageAuto;
				if (pageAuto == stacks)
					lcdSetText(uart1, 2, "stacks");
				else if (pageAuto == intercept)
					lcdSetText(uart1, 2, "intercept");
				else if (pageAuto == preloads)
					lcdSetText(uart1, 2, "preloads only");
				else if (pageAuto == hoard)
					lcdSetText(uart1, 2, "hoarding");
				else if (pageAuto == skills)
					lcdSetText(uart1, 2, "skills");
				else if (pageAuto == nothing)
					lcdSetText(uart1, 2, "do nothing");
				autoSelected = 1;
			}
		}
		//move on to select color
		else{
			if(pageColor == red)
				lcdSetText(uart1,1,"red");
			else
				lcdSetText(uart1,1,"blue");

			//switch color
			if(lcdReadButtons(uart1) == LCD_BTN_RIGHT){
				pageColor = -1*pageColor;
				while(lcdReadButtons(uart1) == LCD_BTN_RIGHT)
					delay(20);
			}

			//go back
			if(lcdReadButtons(uart1) == LCD_BTN_LEFT)
				autoSelected = 0;

			//save selection
			autonomousColor = pageColor;
		}
		delay(20);
	}
}

//our encoders
Encoder yellowFlywheelEncoder;
Encoder greenFlywheelEncoder;

Encoder yellowDriveEncoder;
Encoder greenDriveEncoder;

void initialize() {
	//initialize base encoders
	yellowDriveEncoder = encoderInit(8, 9, true);
	greenDriveEncoder = encoderInit(6, 7, false);

	//initialize flywheel encoders
	yellowFlywheelEncoder = encoderInit(3, 4, false);
	greenFlywheelEncoder = encoderInit(11, 12, false);

	tbhStarted = 0;

	//initialize lcd screen
	lcdInit(uart1 );
	lcdClear(uart1 );
	lcdSetBacklight(uart1, true);

	//run autonomous selection menu when we are done initializing
	lcdAutoSelection();
}
