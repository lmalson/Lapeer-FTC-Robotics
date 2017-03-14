/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.lapeerftcrobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TestServoIntake extends OpMode {


	Servo leftIntakeServo;
	Servo rightIntakeServo;
	Servo armRotationServo;
	Servo armSlideServo;
	Servo armAngleServo;

	/**
	 * Constructor
	 */
	public TestServoIntake() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		leftIntakeServo = hardwareMap.servo.get("servo_1");
		rightIntakeServo = hardwareMap.servo.get("servo_2");
		armRotationServo = hardwareMap.servo.get("servo_3");
		armSlideServo = hardwareMap.servo.get("servo_4");
		armAngleServo = hardwareMap.servo.get("servo_5");

		leftIntakeServo.setPosition(0.5);
		rightIntakeServo.setPosition(0.5);
		armRotationServo.setPosition(0.5);
		armSlideServo.setPosition(0.5);
		armAngleServo.setPosition(0.5);
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		if(gamepad1.left_bumper)
		{
				rightIntakeServo.setPosition(0.0);
				leftIntakeServo.setPosition(1.0);
		}
		else if(gamepad1.right_bumper)
		{
			rightIntakeServo.setPosition(1.0);
			leftIntakeServo.setPosition(0.0);
		}
		else
		{
			rightIntakeServo.setPosition(0.5);
			leftIntakeServo.setPosition(0.5);
		}

		double rotation = (0.5 * gamepad1.right_stick_x) + 0.5;
		double slide = (0.5 * gamepad1.left_stick_x) + 0.5;
		double angle = (0.5 * gamepad1.left_stick_y) + 0.5;

		armRotationServo.setPosition(rotation);
		armSlideServo.setPosition(slide);
		armAngleServo.setPosition(angle);

		telemetry.addData("1", "rotate: " + String.format("%.2f", rotation));
		telemetry.addData("2", "slide: " + String.format("%.2f", slide));
		telemetry.addData("3", "angle: " + String.format("%.2f", angle));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
