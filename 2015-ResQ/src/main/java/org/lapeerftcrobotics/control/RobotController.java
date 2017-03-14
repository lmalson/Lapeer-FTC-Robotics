package org.lapeerftcrobotics.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.lapeerftcrobotics.log.FileLogger;

/**
 * Created by t6810sm on 11/21/2015.
 */
public class RobotController {

    public final static double GRABBER_ARM_ANGLE_UP = 0.685;
    public final static double GRABBER_ARM_ANGLE_DOWN = 0.02;
    public final static double RIGHT_GRABBER_CLOSED = 0.15;
    public final static double RIGHT_GRABBER_OPEN = 0.5;

    private final static double DRIVE_MOTOR_POWER_RAMP_RATE = 0.03; // 0.04
    private final static double WINCH_MOTOR_POWER_RAMP_RATE = 0.05;
    private final static double WINCH_CR_SERVO_RAMP_RATE = 0.05;

    private double leftDriveMotorPowerTarget = 0.0;
    private double leftDriveMotorPower = 0.0;
    private double rightDriveMotorPowerTarget = 0.0;
    private double rightDriveMotorPower = 0.0;
    private double winchMotorPowerTarget = 0.0;
    private double winchMotorPower = 0.0;
    private double winchAngleServoPowerTarget = 0.0;
    private double winchAngleServoPower = 0.0;
    private double armRotateMotorPower = 0.0;

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;
    private DcMotor winchMotor;
    private DcMotor armRotateMotor;
    private Servo winchAngleServo;

    private double rightAllClearArmServoPower = 0.0;
    private double leftAllClearArmServoPower = 0.0;
    private double armAngleServoPos = GRABBER_ARM_ANGLE_DOWN; // Down     0.685 Up
    private double rightGrabberServoPos = RIGHT_GRABBER_CLOSED; // Closed  0.5 Open
    private double armSlideServoPower = 0.0;

    private FileLogger fileLogger;
    private GyroSensor gyroSensor = null;

    private Servo rightAllClearArmServo;
    private Servo leftAllClearArmServo;
    private Servo armAngleServo;
    private Servo armSlideServo;
    private Servo rightGrabberServo;

    public RobotController() {

    }

    public void setGyroSensor(GyroSensor gs) {
        this.gyroSensor = gs;
    }

    public GyroSensor getGyroSensor() {
        return this.gyroSensor;
    }
    
    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public void setLeftDriveMotor(DcMotor m) {
        this.leftDriveMotor = m;
    }

    public void setRightDriveMotor(DcMotor m) {
        this.rightDriveMotor = m;
    }

    public void setWinchMotor(DcMotor m) {
        this.winchMotor = m;
    }

    public void setWinchAngleServo(Servo s) { this.winchAngleServo = s; }

    public void setLeftDriveMotorPowerTarget(double t) {
        this.leftDriveMotorPowerTarget = t;
    }

    public void setRightDriveMotorPowerTarget(double t) {
        this.rightDriveMotorPowerTarget = t;
    }

    public void setWinchMotorPowerTarget(double t) {
        this.winchMotorPowerTarget = t;
    }

    public void setWinchAngleServoPowerTarget( double t) {
        this.winchAngleServoPowerTarget = t;
    }

    public void processFast() {
    }

    public void init() {
    }

    public void process() {

        leftDriveMotorPower = leftDriveMotorPowerTarget;
        rightDriveMotorPower = rightDriveMotorPowerTarget;

        winchMotorPower = rateLimitValue(winchMotorPowerTarget,winchMotorPower,WINCH_MOTOR_POWER_RAMP_RATE,false);
        winchAngleServoPower = rateLimitValue(winchAngleServoPowerTarget,winchAngleServoPower,WINCH_CR_SERVO_RAMP_RATE,true);

//        leftButtonServoPos = rateLimitValue(targetLeftButtonServoPos,leftButtonServoPos, BUTTON_SERVO_RAMP_RATE,true);

        Range.clip(leftDriveMotorPower, -1.0, 1.0);
        Range.clip(rightDriveMotorPower, -1.0, 1.0);
        Range.clip(winchMotorPower, -1.0, 1.0);
        Range.clip(armRotateMotorPower, -1.0, 1.0);

        leftDriveMotor.setPower(-leftDriveMotorPower);    // -left
        rightDriveMotor.setPower(rightDriveMotorPower);
        winchMotor.setPower(-winchMotorPower);
        armRotateMotor.setPower(armRotateMotorPower);
        winchAngleServo.setPosition(0.5 + winchAngleServoPower);

        rightAllClearArmServo.setPosition(0.5 + rightAllClearArmServoPower);
        leftAllClearArmServo.setPosition(0.5 + leftAllClearArmServoPower);
        armAngleServo.setPosition(armAngleServoPos);
        armSlideServo.setPosition(0.5 + armSlideServoPower);
        rightGrabberServo.setPosition(rightGrabberServoPos);

        if (fileLogger != null)
            fileLogger.writeEvent("robotController.process()","MotPwr lft: "+leftDriveMotorPower+" rgt: "+rightDriveMotorPower);
    }

    private double rateLimitValue(double target, double power, double rate, boolean servo) {
        double command = power;
        if (target > command) {
            command += rate;
            if (command > target)
                command = target;
        } else {
            command -= rate;
            if (command < target)
                command = target;
        }

        if (command > 1.0) {
            command = 1.0;
        } else if ((servo) && (command < 0.0)) {
            command = 0.0;
        } else if (command < -1.0) {
            command = -1.0;
        }


        return command;
    }

    public void setArmRotateMotor(DcMotor motor_4) {
        armRotateMotor = motor_4;
    }

    public void setLeftAllClearArmServo(Servo servo_1) {
        this.leftAllClearArmServo = servo_1;
    }

    public void setRightAllClearArmServo(Servo servo_2) {
        this.rightAllClearArmServo = servo_2;
    }

    public void setArmAngleServo(Servo s) {
        this.armAngleServo = s;
    }

    public void setArmSlideServo(Servo servo_5) {
        this.armSlideServo = servo_5;
    }

    public void setRightGrabberServo(Servo s) {
        this.rightGrabberServo = s;
    }

    public void setRightAllClearArmServoPower(double v) {
        this.rightAllClearArmServoPower = v;
    }

    public void setLeftAllClearArmServoPower(double v) {
        this.leftAllClearArmServoPower = v;
    }

    public void setArmAngleServoPos(double p) {
        this.armAngleServoPos = p;
    }

    public void setRightGrabberServoPos(double p) {
        this.rightGrabberServoPos = p;
    }

    public void setArmSliderServoPower(double v) {
        this.armSlideServoPower = v;
    }

    public void setArmRotatePower(double p) {
        this.armRotateMotorPower = p;
    }

    public void setRotateServo(Servo servo_4) {
    }
}
