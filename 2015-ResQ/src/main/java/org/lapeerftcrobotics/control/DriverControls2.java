package org.lapeerftcrobotics.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by SYSTEM on 10/9/2015.
 */
public class DriverControls2 {

    private Gamepad driverGamepad;
    private Gamepad operatorGamepad;
    private float driveLeftPower = 0;
    private float driveRightPower = 0;
    private float winchExtendPower = 0;
    private float winchAnglePower = 0;
    private double rightGrabberServoPos = 0.5;
    private double leftGrabberServoPos = 0.5;
    private double armAngleServoPos = 0.5;
    private double rotationPower = 0.5;
    private double rotationPowerAdj = 0.0;
    private double slidePower = 0.5;
    private boolean autoPickupForward = false;
    private boolean autoPickupBackward = false;
    private boolean unload = false;
    private boolean armBrakeOn = false;

    private boolean testAutonReset = false;
    private boolean testAutonRaise = false;
    private boolean testAutonRelease = false;

    /**
     *
     */
    public DriverControls2() {
    }

    public void setDriverGamepad(Gamepad driverGamepad) {
        this.driverGamepad = driverGamepad;
    }

    public void setOperatorGamepad(Gamepad operatorGamepad) {
        this.operatorGamepad = operatorGamepad;
    }

    public void init() {

    }

    /**
     *
     */
    public void process() {

        // note that if y equal -1 then joystick is pushed all of the way forward.
        float driverLeft = -driverGamepad.left_stick_y;
        float driverRight = -driverGamepad.right_stick_y;
        float operatorLeft = -operatorGamepad.left_stick_y;
        float operatorRight = -operatorGamepad.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        driverRight = Range.clip(driverRight, -1.0F, 1.0F);
        driverLeft = Range.clip(driverLeft, -1.0F, 1.0F);
        operatorRight = Range.clip(operatorRight, -1.0F, 1.0F);
        operatorLeft = Range.clip(operatorLeft, -1.0F, 1.0F);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        this.driveRightPower = (float)scaleInput(driverRight);
        this.driveLeftPower =  (float)scaleInput(driverLeft);
        this.winchAnglePower = operatorLeft; // (float)scaleInput(operatorLeft);
        this.winchExtendPower =  operatorRight; // scaleInput(operatorRight);

        float driverTrigger = driverGamepad.right_trigger - driverGamepad.left_trigger;
        driverTrigger = Range.clip(driverTrigger, -1, 1);

        autoPickupForward = operatorGamepad.b;
        autoPickupBackward = operatorGamepad.x;

        unload = operatorGamepad.a;

        testAutonReset = driverGamepad.y;
        testAutonRaise = driverGamepad.b;
        testAutonRelease = driverGamepad.a;

        if (driverGamepad.dpad_left) {
            rotationPower = 0.2; // 0.0
        }
        else if (driverGamepad.dpad_right) {
            rotationPower = 0.8; // 1.0
        }
        else {
            rotationPower = 0.5;
        }

        if (driverGamepad.left_trigger > 0.0) {
            rotationPower = 0.5 - (0.39*driverGamepad.left_trigger);
        }
        if (driverGamepad.right_trigger > 0.0) {
            rotationPower = 0.5 + (0.39*driverGamepad.right_trigger);
        }

        if (driverGamepad.dpad_up || operatorGamepad.dpad_up) {
            slidePower = 0.9;
        }
        else if (driverGamepad.dpad_down || operatorGamepad.dpad_down) {
            slidePower = 0.1;
        }
        else {
            slidePower = 0.5;
        }

    }

    public boolean getTestAutonReset() {
        return this.testAutonReset;
    }

    public boolean getTestAutonRaise() {
        return this.testAutonRaise;
    }

    public boolean getTestAutonRelease() {
        return this.testAutonRelease;
    }

    public boolean isArmBrakeOn() {
        return this.armBrakeOn;
    }

    public double getDriveLeftPower() {
        return this.driveLeftPower;
    }

    public double getDriveRightPower() {
        return this.driveRightPower;
    }

    public double getWinchExtendPower() {
        return this.winchExtendPower;
    }

    public double getWinchAnglePower() {
        return this.winchAnglePower;
    }

    public boolean getAutoPickupForward() {
        return this.autoPickupForward;
    }

    public boolean getAutoPickupBackward() {
        return this.autoPickupBackward;
    }

    public boolean getUnload() {
        return this.unload;
    }


    public double getRotationPower() {
        return this.rotationPower;
    }

    public double getRotationMotorPower() {
        return 0.9*(this.rotationPower-0.5);
    }

    public double getSlidePower() {
        return this.slidePower;
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    private double scaleInput(double dVal)  {
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
