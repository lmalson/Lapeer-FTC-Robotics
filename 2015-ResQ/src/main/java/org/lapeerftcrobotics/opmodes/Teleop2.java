package org.lapeerftcrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.lapeerftcrobotics.control.AutoPickup;
import org.lapeerftcrobotics.control.DriverControls2;


/**
 * Created by t6810sm on 11/21/2015.
 */
public class Teleop2 extends OpMode {

    DriverControls2 driverControls;
    AutoPickup autoPickup;

    DcMotor driveLeftMotor; // motor_1
    DcMotor driveRightMotor; // motor_2
    DcMotor winchMotor; // motor_3
    DcMotor rotateMotor; // motor_4

    Servo rightAllClearArmServo; // servo_1 CR Servo
    Servo leftAllClearArmServo; // servo_2 CR Servo
    Servo winchAngleServo; // servo_3 CR Servo
    Servo armRotationServo; // servo_4 CR Servo
    Servo armSlideServo; // servo_5 CR Servo
    Servo rightGrabberServo; // servo_6
    Servo leftGrabberServo; // servo_7
    Servo armAngleServo; // servo_8
    Servo armBrakeServo; // servo_9

    GyroSensor sensorGyro;

    long winchMotorPos = 0;

    double leftMotorPower;
    double rightMotorPower;
    double winchMotorPower;
    double rotateMotorPower;

    int heading = 0;

    public Teleop2() {
        super();
        driverControls = new DriverControls2();
        autoPickup = new AutoPickup();
    }

    @Override
    public void init() {

        driveLeftMotor = hardwareMap.dcMotor.get("motor_1");
        driveRightMotor = hardwareMap.dcMotor.get("motor_2");
        winchMotor = hardwareMap.dcMotor.get("motor_3");
        rotateMotor = hardwareMap.dcMotor.get("motor_4");

        rightAllClearArmServo = hardwareMap.servo.get("servo_1");
        leftAllClearArmServo = hardwareMap.servo.get("servo_2");
        winchAngleServo = hardwareMap.servo.get("servo_3");
        armRotationServo = hardwareMap.servo.get("servo_4");
        armSlideServo = hardwareMap.servo.get("servo_5");
        rightGrabberServo = hardwareMap.servo.get("servo_6");
        leftGrabberServo = hardwareMap.servo.get("servo_7");
        armAngleServo = hardwareMap.servo.get("servo_8");
        armBrakeServo = hardwareMap.servo.get("servo_9");

        driverControls.init();

        rightGrabberServo.setPosition(AutoPickup.RIGHT_GRABBER_CLOSED);
        armAngleServo.setPosition(AutoPickup.SERVO_ARM_DOWN);
        winchAngleServo.setPosition(0.5);
        armRotationServo.setPosition(0.5);
        armSlideServo.setPosition(0.5);
        rightAllClearArmServo.setPosition(0.5);
        leftAllClearArmServo.setPosition(0.5);
        armBrakeServo.setPosition(1.0); // initial brake position

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        sensorGyro.calibrate();
    }

    @Override
    public void loop() {

        if (sensorGyro.isCalibrating()) {
            telemetry.addData("Gyro Calibrating...","");
            return;
        }
        heading = sensorGyro.getHeading();

        driverControls.setDriverGamepad(gamepad1);
        driverControls.setOperatorGamepad(gamepad2);
        driverControls.process();

        autoPickup.setRunForward(driverControls.getAutoPickupForward());
        autoPickup.setRunBackward(driverControls.getAutoPickupBackward());
        autoPickup.setUnload(driverControls.getUnload());

        autoPickup.setTestAutonReset(driverControls.getTestAutonReset());
        autoPickup.setTestAutonRaise(driverControls.getTestAutonRaise());
        autoPickup.setTestAutonRelease(driverControls.getTestAutonRelease());

        autoPickup.process();

        rightGrabberServo.setPosition(autoPickup.getRightGrabberServoPos());
        armAngleServo.setPosition(autoPickup.getArmAngleServoPos());

        leftMotorPower = -driverControls.getDriveLeftPower();
        driveLeftMotor.setPower(leftMotorPower);
        rightMotorPower = driverControls.getDriveRightPower();
        driveRightMotor.setPower(rightMotorPower);
        winchMotorPower = -driverControls.getWinchExtendPower();
        winchMotor.setPower(winchMotorPower);

        rotateMotor.setPower(driverControls.getRotationMotorPower());

        // winch gain
        double gain = 0.025;
        if (gamepad2.a)
            gain = 0.05;
        else if (gamepad2.b)
            gain = 0.0375;
        else if (gamepad2.x)
            gain = 0.5;

        winchAngleServo.setPosition(Range.clip(0.5 + gain * driverControls.getWinchAnglePower(), 0.0, 1.0));

        armRotationServo.setPosition(driverControls.getRotationPower());
        armSlideServo.setPosition(driverControls.getSlidePower());

        if (driverControls.isArmBrakeOn()) {
            armBrakeServo.setPosition(0.0);
        } else {
            armBrakeServo.setPosition(1.0);
        }

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
            rightAllClearArmServo.setPosition(0.2); // .2 forward
        }
        else {
            rightAllClearArmServo.setPosition(0.5); // stop
        }
        telemetry.addData("1 Heading: ", String.format("%03d  brake: %2.2f", heading, armBrakeServo.getPosition()));
//        telemetry.addData("1","motor l/r pwr: "+String.format("%2.2f %2.2f",leftMotorPower,rightMotorPower));
        telemetry.addData("2 Auto: ",String.format("pickup state: %d", autoPickup.getState()));
//        telemetry.addData("3","winch pwr: "+String.format("%.2f",winchMotorPower));

        telemetry.addData("4", "auto pickup fwd: " + driverControls.getAutoPickupForward());
        telemetry.addData("5", "auto pickup rev: " + driverControls.getAutoPickupBackward());
        telemetry.addData("6", "auto pickup unload: " + driverControls.getUnload());

//        telemetry.addData("4","latch servo: "+String.format("%.2f",latchServoPos));
//        telemetry.addData("5","zip r servo: "+String.format("%.2f",zipReleaseServo.getPosition()));
//        telemetry.addData("7","left trigger: "+String.format("%.3f",gamepad2.left_trigger));
//        telemetry.addData("8","right trigger: "+String.format("%.3f",gamepad2.right_trigger));
    }

    @Override
    public void stop() {
    }
}
