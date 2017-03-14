package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 */

//@TeleOp(name="TeleopTest", group="Iterative Opmode")
//hello! -Devin
public class TeleopWithMacro extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor leftMotor2 = null;
    private DcMotor rightMotor2 = null;

    private DcMotor intakeMotor = null;

    private int state = 0;
    private int stateCnt = 0;

    double leftPower = 0.0;
    double rightPower = 0.0;

    int startRpos = 0;
    int startLpos = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor  = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_3");
        leftMotor2  = hardwareMap.dcMotor.get("motor_2");
        rightMotor2 = hardwareMap.dcMotor.get("motor_4");

        startRpos = rightMotor.getCurrentPosition();
        startLpos = leftMotor.getCurrentPosition();

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

        int rpos = rightMotor.getCurrentPosition() - startRpos;
        int lpos = leftMotor.getCurrentPosition() - startLpos;
        int rpos2 = rightMotor2.getCurrentPosition();
        int lpos2 = leftMotor2.getCurrentPosition();

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("RPos ",""+rpos+ " rpos2 "+rpos2);
        telemetry.addData("LPos ",""+lpos+ " lpos2 "+lpos2);
        telemetry.update();

//        double leftPower = gamepad1.left_stick_y;
//        double rightPower = -gamepad1.right_stick_y;


        int nextState = state;

        switch(state) {
            case 0:
//                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftPower = 0.4;
                rightPower = 0.4;
                nextState = 1;
                break;
            case 1:
                if (rpos > 8000 || lpos > 8000 || stateCnt > 50) {
                    leftPower = 0.0;
                    rightPower = 0.0;
                    nextState = 2;
                }
                break;
            case 2:
                if (stateCnt > 5) {
                    leftPower = -0.4;
                    rightPower = -0.4;
                    nextState = 3;
                }
                break;
            case 3:
                if (rpos < 1 || lpos < 1 || stateCnt > 50) {
                    leftPower = 0.0;
                    rightPower = 0.0;
                    nextState = 4;
                }
        }

        if (state != nextState) {
            state = nextState;
            stateCnt = 0;
        }
        else {
            stateCnt++;
        }


        leftMotor.setPower(leftPower);
        leftMotor2.setPower(leftPower);
        rightMotor.setPower(rightPower);
        rightMotor2.setPower(rightPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
