package org.lapeerftcrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.lapeerftcrobotics.control.DriverControls;
import org.lapeerftcrobotics.control.HiTechnicMotorController;


/**
 * Created by t6810sm on 11/21/2015.
 */
public class Teleop extends OpMode {

    DriverControls driverControls;

    DcMotor driveLeftMotor; // motor_1
    DcMotor driveRightMotor; // motor_2
    DcMotor winchMotor; // motor_3
//    DcMotorController motorController2;
//    HiTechnicMotorController hMotorController2;

    Servo winchAngleServo; // CR Servo
    Servo leftAllClearArmServo; // CR Servo
    Servo rightAllClearArmServo; // CR Servo
    Servo peopleBoxServo;
    Servo latchServo;
    Servo zipReleaseServo; // CR Servo

    long winchMotorPos = 0;

    double peopleBoxServoPos;
    double latchServoPos;
    double winchAngleServoPos;

    double leftMotorPower;
    double rightMotorPower;
    double winchMotorPower;

    public Teleop() {
        super();
        driverControls = new DriverControls();
    }

    @Override
    public void init() {

        driveLeftMotor = hardwareMap.dcMotor.get("motor_1");
        driveRightMotor = hardwareMap.dcMotor.get("motor_2");
        winchMotor = hardwareMap.dcMotor.get("motor_3");

        winchAngleServo = hardwareMap.servo.get("servo_1");
        peopleBoxServo = hardwareMap.servo.get("servo_2");
        rightAllClearArmServo = hardwareMap.servo.get("servo_3");
        leftAllClearArmServo = hardwareMap.servo.get("servo_4");
        latchServo = hardwareMap.servo.get("servo_5");
        zipReleaseServo = hardwareMap.servo.get("servo_6");

        driverControls.init();

        peopleBoxServoPos = 0.18;
        latchServoPos = 1.0;

    }

    @Override
    public void loop() {

        driverControls.setDriverGamepad(gamepad1);
        driverControls.setOperatorGamepad(gamepad2);
        driverControls.process();

        leftMotorPower = -driverControls.getDriveLeftPower();
        driveLeftMotor.setPower(leftMotorPower);
        rightMotorPower = driverControls.getDriveRightPower();
        driveRightMotor.setPower(rightMotorPower);
        winchMotorPower = -driverControls.getWinchExtendPower();
        winchMotor.setPower(winchMotorPower);

        // winch gain
        double gain = 0.025;
        if (gamepad2.a)
            gain = 0.05;
        else if (gamepad2.b)
            gain = 0.0375;
        else if (gamepad2.x)
            gain = 0.5;

        winchAngleServoPos = 0.5 + gain*driverControls.getWinchAnglePower();   // 0.05
        Range.clip(winchAngleServoPos, 0.0, 1.0);
        winchAngleServo.setPosition(winchAngleServoPos);

        // people box
        if (gamepad2.dpad_up) {
            peopleBoxServoPos += 0.025;
            if (peopleBoxServoPos > 1.0)
                peopleBoxServoPos = 1.0;
        }
        else if (gamepad2.dpad_down) {
            peopleBoxServoPos -= 0.025;
            if (peopleBoxServoPos < 0.0)
                peopleBoxServoPos = 0.0;
        }

        peopleBoxServo.setPosition(peopleBoxServoPos);

        // left & right all clear servo

        if (gamepad2.left_bumper) {
            leftAllClearArmServo.setPosition(0.4); // return
        }
        else if (gamepad2.left_trigger > 0.0) {
            leftAllClearArmServo.setPosition(0.8); // forward
        }
        else {
            leftAllClearArmServo.setPosition(0.5); // stop
        }

        if (gamepad2.right_bumper) {
            rightAllClearArmServo.setPosition(0.6); // return
        }
        else if (gamepad2.right_trigger > 0.0) {
            rightAllClearArmServo.setPosition(0.2); // forward
        }
        else {
            rightAllClearArmServo.setPosition(0.5); // stop
        }

        telemetry.addData("1","left pwr: "+String.format("%.2f",leftMotorPower));
        telemetry.addData("2","right pwr: "+String.format("%.2f",rightMotorPower));
        telemetry.addData("3","winch pwr: "+String.format("%.2f",winchMotorPower)+" angle pos: "+String.format("%.2f",winchAngleServoPos));
//        telemetry.addData("4","latch servo: "+String.format("%.2f",latchServoPos));
//        telemetry.addData("5","zip r servo: "+String.format("%.2f",zipReleaseServo.getPosition()));
        telemetry.addData("6","people servo: "+String.format("%.2f",peopleBoxServoPos));
        telemetry.addData("7","left trigger: "+String.format("%.3f",gamepad2.left_trigger));
        telemetry.addData("8","right trigger: "+String.format("%.3f",gamepad2.right_trigger));
    }

    @Override
    public void stop() {
    }
}
