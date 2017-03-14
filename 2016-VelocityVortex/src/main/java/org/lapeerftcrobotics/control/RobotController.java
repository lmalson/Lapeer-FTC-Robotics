package org.lapeerftcrobotics.control;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 *
 */
public class RobotController {

    public final static boolean EMULATED = false; // false
    public final static int ROBOT_7138 = 0;
    public final static int ROBOT_8935 = 1;

    public final static double ROTATE_KP_8935 = 0.037; // 0.035
    public final static double ROTATE_KD_8935 = 0.07; // .1
    public final static double ROTATE_KP_7138 = 0.04; // 0.03
    public final static double ROTATE_KD_7138 = 0.07; //
    public final static double HOPPER_SERVO_OPEN = 0.5;
    public final static double HOPPER_SERVO_CLOSED = 1.0;
    public final static double LEFT_BUTTON_SERVO_OUT_8935 = 0.2;
    public final static double LEFT_BUTTON_SERVO_IN_8935 = 0.5;
    public final static double RIGHT_BUTTON_SERVO_OUT_8935 = 0.8;
    public final static double RIGHT_BUTTON_SERVO_IN_8935 = 0.5;

    public final static double LEFT_BUTTON_SERVO_OUT_7138 = 1.0;
    public final static double LEFT_BUTTON_SERVO_IN_7138 = 0.71;   // 0.7
    public final static double RIGHT_BUTTON_SERVO_OUT_7138 = 0.0;
    public final static double RIGHT_BUTTON_SERVO_IN_7138 = 0.33; // 0.3

    public final static double CANNON_ON = 0.95;
    private final static double DRIVE_THROTTLE_RAMP_RATE = 0.06; //
    private final static double DRIVE_ANGLE_RAMP_RATE = 0.07; //

    private int robot = -1;

    private double driveThrottleTarget = 0.0;
    private double driveThrottle = 0.0;
    private double driveAngleTarget = 0.0;
    private double driveAngle = 0.0;
    private double rightPower = 0.0;
    private double leftPower = 0.0;

    private double strikerDistance = 0.0;
    private double prevStrikerDistance = 0.0;
    private double deltaStrikerDistance = 0.0;

    private double hopperServoTargetPos = HOPPER_SERVO_CLOSED;
    private double leftButtonServoTargetPos = 0.5;
    private double rightButtonServoTargetPos = 0.5;

    private boolean isCannonOn = false;

    private DcMotor frontLeftDriveMotor;
    private DcMotor rearLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor rearRightDriveMotor;
    private DcMotor intakeMotor = null;
    private DcMotor cannonMotor = null;
    private OpticalDistanceSensor odsSensor;  // Hardware Device Object
    private Servo hopperServo = null;
    private Servo leftButtonServo = null;
    private Servo rightButtonServo = null;
    private GyroSensor gyroSensor = null;
    private ModernRoboticsI2cRangeSensor rangeSensor = null;

    private double targetHeading = 0.0;
    private boolean isAutoHeading = false;
    private double headingError = 0.0;
    private double deltaHeadingError = 0.0;
    private double heading = 0.0;
    private double distance = 0.0;

    private int leftButtonCnt = 0;
    private int rightButtonCnt = 0;
    private boolean leftButtonOut = false;
    private boolean rightButtonOut = false;

    public RobotController() {

    }

    public void setRobot(int r) {
        this.robot = r;
    }

    public GyroSensor getGyroSensor() {
        return this.gyroSensor;
    }

    public void setDriveThrottleTarget(double t) {
        this.driveThrottleTarget = t;
    }

    public double getDriveThrottleTarget() {
        return this.driveThrottleTarget;
    }

    public void setDriveAngleTarget(double t) {
        this.driveAngleTarget = t;
    }

    public double getDriveAngle() {
        return driveAngle;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public double getStrikerDistance() {
        return strikerDistance;
    }

    public double getDeltaStrikerDistance() {
        return deltaStrikerDistance;
    }

    public void setHopperServoTargetPos(double t) {
        hopperServoTargetPos = t;
    }

    public void openHopperDoor() {
        hopperServoTargetPos = HOPPER_SERVO_OPEN;
    }

    public void closeHopperDoor() {
        hopperServoTargetPos = HOPPER_SERVO_CLOSED;
    }

    public void setLeftButtonServoTargetPos(double t) {
        leftButtonServoTargetPos = t;
    }
    public void setRightButtonServoTargetPos(double t) {
        rightButtonServoTargetPos = t;
    }

    public void pressLeftButton() {
        leftButtonCnt = 20;
    } // 20

    public void leftButtonOut() {
        leftButtonOut = true;
    }

    public void leftButtonIn() {
        leftButtonOut = false;
    }

    public void rightButtonOut() {
        rightButtonOut = true;
    }

    public void rightButtonIn() {
        rightButtonOut = false;
    }

    public void pressRightButton() {
        rightButtonCnt = 20;
    } // 20

    public void setIsCannonOn(boolean b) {
        isCannonOn = b;
    }

    public void setTargetHeading(double t) {
        targetHeading = t;
    }

    public double getTargetHeading() { return this.targetHeading; }

    public void setIsAutoHeading(boolean b) {
        isAutoHeading = b;
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getDistance() {return distance;}

    public double getHeading() {
        if (EMULATED) {
            return -90.0;
        }
        else {
            heading = gyroSensor.getHeading();
            if (heading > 180.0)
                heading -= 360.0;
            else if (heading < -180.0)
                heading += 360.0;
            return heading;
        }
    }

    public boolean isGyroCalibrating() {
        if (EMULATED)
            return false;
        else
            return gyroSensor.isCalibrating();
    }

    public void processFast() {
    }

    public void init(HardwareMap hardwareMap) {
        if (!EMULATED) {
            frontLeftDriveMotor = hardwareMap.dcMotor.get("flm");
            frontRightDriveMotor = hardwareMap.dcMotor.get("frm");
            rearLeftDriveMotor = hardwareMap.dcMotor.get("rlm");
            rearRightDriveMotor = hardwareMap.dcMotor.get("rrm");
            intakeMotor = hardwareMap.dcMotor.get("im");
            cannonMotor = hardwareMap.dcMotor.get("sm");
            odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
            hopperServo = hardwareMap.servo.get("hs");
            hopperServo.setPosition(hopperServoTargetPos);
            leftButtonServo = hardwareMap.servo.get("lbs");
            rightButtonServo = hardwareMap.servo.get("rbs");
            rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rs");

            if (robot == ROBOT_7138) {
                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_IN_7138;
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_IN_7138;
            }
            else if (robot == ROBOT_8935) {
                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_IN_8935;
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_IN_8935;
            }

            leftButtonServo.setPosition(leftButtonServoTargetPos);
            rightButtonServo.setPosition(rightButtonServoTargetPos);

            gyroSensor = hardwareMap.gyroSensor.get("gs");
            // calibrate the gyro.
            gyroSensor.calibrate();
        }
    }

    public void process() {

        double range = rangeSensor.getDistance(DistanceUnit.CM);
        if ( range < 255.0)
            distance = range;

        if (robot == ROBOT_7138) {
            if (leftButtonCnt > 0 || leftButtonOut) {
                leftButtonCnt--;
                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_OUT_7138;
            }
            else {
                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_IN_7138;
            }

            if (rightButtonCnt > 0 || rightButtonOut) {
                rightButtonCnt--;
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_OUT_7138;
            }
            else {
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_IN_7138;
            }
        }
        else if (robot == ROBOT_8935) {
            if (leftButtonCnt > 0 || leftButtonOut) {
                leftButtonCnt--;

                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_OUT_8935;
            }
            else {
                leftButtonServoTargetPos = LEFT_BUTTON_SERVO_IN_8935;
            }

            if (rightButtonCnt > 0 || rightButtonOut) {
                rightButtonCnt--;
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_OUT_8935;
            }
            else {
                rightButtonServoTargetPos = RIGHT_BUTTON_SERVO_IN_8935;
            }
        }

        if (isAutoHeading) {
            double prevHeadingError = headingError;
            headingError = targetHeading - heading;
            if (headingError < -180.0) {
                headingError += 360.0;
            }
            else if (headingError > 180.0) {
                headingError -= 360.0;
            }

            deltaHeadingError = headingError - prevHeadingError;
            double adj = 0.0;

            if (robot == ROBOT_7138) {
                adj = ROTATE_KP_7138 * headingError + ROTATE_KD_7138 * deltaHeadingError;
            }
            else if (robot == ROBOT_8935) {
                adj = ROTATE_KP_8935 * headingError + ROTATE_KD_8935 * deltaHeadingError;
            }

            if (adj > 0.5)
                adj = 0.5;
            else if (adj < -0.5 )
                adj = -0.5;
            driveAngle = adj;
            driveThrottle = driveThrottleTarget;
        } else {
            driveAngle = rateLimitValue(driveAngleTarget, driveAngle, DRIVE_ANGLE_RAMP_RATE, false);
            driveThrottle = rateLimitValue(driveThrottleTarget, driveThrottle, DRIVE_THROTTLE_RAMP_RATE, false);
            headingError = 0.0;
            deltaHeadingError = 0.0;
        }

        rightPower = -driveThrottle - driveAngle;
        leftPower = -driveThrottle + driveAngle;

        Range.clip(rightPower, -1.0, 1.0);
        Range.clip(leftPower, -1.0, 1.0);

        if (EMULATED) {
            strikerDistance = 0.0;
            deltaStrikerDistance = 0.01;
            prevStrikerDistance = strikerDistance;
        }
        else {
            rearLeftDriveMotor.setPower(-leftPower);
            rearRightDriveMotor.setPower(rightPower);
            frontLeftDriveMotor.setPower(-leftPower);
            frontRightDriveMotor.setPower(rightPower);
            if (isCannonOn)
                cannonMotor.setPower(CANNON_ON);
            else
                cannonMotor.setPower(0.0);

            intakeMotor.setPower(0.0);

            hopperServo.setPosition(hopperServoTargetPos);
            leftButtonServo.setPosition(leftButtonServoTargetPos);
            rightButtonServo.setPosition(rightButtonServoTargetPos);

            strikerDistance = odsSensor.getLightDetected();
            deltaStrikerDistance = strikerDistance - prevStrikerDistance;
            prevStrikerDistance = strikerDistance;
        }
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

}