/*
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 */
@TeleOp(name="Teleop_8935", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class Teleop_8935 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor rearLeftMotor = null;
    private DcMotor rearRightMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor shooterMotor = null;
    private OpticalDistanceSensor odsSensor;  // Hardware Device Object

    private Servo hopperServo = null;
    private Servo leftButtonServo = null;
    private Servo rightButtonServo = null;
    private GyroSensor gyro = null;

    double leftPower = 0.0;
    double rightPower = 0.0;
    boolean shoot = false;
    boolean intakeForward = false;
    boolean intakeReverse = false;

    double prevShooterDist = 0.0;
    double maxShooterDist = 0.0;

//    boolean autoShoot = false;

    int cannonState = 0;
    double autoLoadTimer = 0.0;
//    double autoLoadDetectTimer = 0.0;

    boolean tankOverHaloDrive = false;
    boolean inverseDrive = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeftMotor  = hardwareMap.dcMotor.get("flm");
        frontRightMotor = hardwareMap.dcMotor.get("frm");
        rearLeftMotor = hardwareMap.dcMotor.get("rlm");
        rearRightMotor = hardwareMap.dcMotor.get("rrm");
        intakeMotor = hardwareMap.dcMotor.get("im");
        shooterMotor = hardwareMap.dcMotor.get("sm");
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        hopperServo = hardwareMap.servo.get("hs");
        hopperServo.setPosition(1.0);
        leftButtonServo = hardwareMap.servo.get("lbs");
        leftButtonServo.setPosition(0.5);
        rightButtonServo = hardwareMap.servo.get("rbs");
        rightButtonServo.setPosition(0.5);

        gyro = hardwareMap.gyroSensor.get("gs");
        gyro.calibrate();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        if (gyro.isCalibrating()) {
//            telemetry.addData("Gyro Calibrating...","");
//            return;
//        }

        double deltaDist = odsSensor.getLightDetected() - prevShooterDist;
        prevShooterDist = odsSensor.getLightDetected();
        if(deltaDist > maxShooterDist)
            maxShooterDist = deltaDist;

        double leftPower = 0.0;
        double rightPower = 0.0;
        boolean shoot = gamepad2.a;
        boolean intakeForward = gamepad2.right_bumper;
        boolean intakeReverse = gamepad2.left_bumper;

       if (gamepad1.y) {
           tankOverHaloDrive = !tankOverHaloDrive;
     }

        if (gamepad1.left_bumper)
            inverseDrive = true;
        else
            inverseDrive = false;

        if (tankOverHaloDrive) { // TANK DRIVE
            if (gamepad1.left_bumper) { // INVERSE
                leftPower = gamepad1.right_stick_y;
                rightPower = gamepad1.left_stick_y;
            } else { // NORMAL
                leftPower = -gamepad1.left_stick_y;
                rightPower = -gamepad1.right_stick_y;
            }
        }
        else { // HALO DRIVE
            float throttle = -gamepad1.left_stick_y;
            float direction = -gamepad1.right_stick_x;
            if (gamepad1.left_bumper) { // INVERSE
                rightPower = -throttle + direction;
                leftPower = -throttle - direction;
            }
            else {
                rightPower = throttle + direction;
                leftPower = throttle - direction;
            }
        }

        rearLeftMotor.setPower(-leftPower);
        rearRightMotor.setPower(rightPower);
        frontLeftMotor.setPower(-leftPower);
        frontRightMotor.setPower(rightPower);

        if (gamepad1.right_trigger > 0.0 || gamepad2.right_trigger > 0.0)
            leftButtonServo.setPosition(0.2);
        else
            leftButtonServo.setPosition(0.5);

        if (gamepad1.left_trigger > 0.0 || gamepad2.left_trigger > 0.0)
            rightButtonServo.setPosition(0.8);
        else
            rightButtonServo.setPosition(0.5);

        switch(cannonState) {
                case 0: // stopped
                    if (gamepad2.b) {
                        hopperServo.setPosition(0.55);
                    }
                    else {
                        hopperServo.setPosition(1.0);
                    }
                    if (gamepad2.a) {
                        shooterMotor.setPower(1.0);
                    }
                    else {
                        shooterMotor.setPower(0.0);
                    }
                    if (gamepad2.y)
                        cannonState = 1;
                    break;
                case 1: // rotating
                    shooterMotor.setPower(1.0);
                    if (deltaDist > 0.0045) { // .005
                        cannonState = 2;
                        autoLoadTimer = runtime.milliseconds() + 500.0;
                    }
                    break;
                case 2: // load
                    hopperServo.setPosition(0.55);
                    if (runtime.milliseconds() > autoLoadTimer)
                        cannonState = 3;
                    break;
                case 3:
                    shooterMotor.setPower(0.0);
                    hopperServo.setPosition(1.0);
                    cannonState = 0;
                    break;
            }


        if(intakeForward){
            intakeMotor.setPower(0.8);
        }
        else if(intakeReverse){
            intakeMotor.setPower(-0.8);
        }
        else{
            intakeMotor.setPower(0.0);
        }

        if (tankOverHaloDrive) {
            telemetry.addData("Drive", "TANK Inv: "+inverseDrive);
        }
        else {
            telemetry.addData("Drive", "HALO Inv: "+inverseDrive);
        }
        telemetry.addData("Status", "Running: " + runtime.toString() + " Heading: " + gyro.getHeading());
        telemetry.addData("Load Timer","Inverse: "+inverseDrive);
//        telemetry.addData("Raw",    odsSensor.getRawLightDetected());
        telemetry.addData("Normal", odsSensor.getLightDetected());
//        telemetry.addData("Delta", deltaDist);
//        telemetry.addData("DeltaMax", maxShooterDist);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
