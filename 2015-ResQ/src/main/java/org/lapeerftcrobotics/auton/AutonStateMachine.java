package org.lapeerftcrobotics.auton;

import com.qualcomm.robotcore.util.Range;

import org.lapeerftcrobotics.camera.OpenCVCameraViewListener;
import org.lapeerftcrobotics.control.Alliance;
import org.lapeerftcrobotics.control.BeaconTargetController;
import org.lapeerftcrobotics.control.RampTargetController;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.log.FileLogger;
import org.lapeerftcrobotics.opmodes.AutonOp;
import org.opencv.core.Rect;

/**
 * Created by t6810sm on 11/21/2015.
 */
public class AutonStateMachine {

    private AutonOp parent = null;

    private int state = -1;
    private int stateCnt;

    private boolean delay = false;

    private Beacon beacon = null;
    private RobotController robotController = null;
    private BeaconTargetController beaconTargetController = null;
    private FileLogger fileLogger = null;
    private Ramp ramp = null;
    private RampTargetController rampTargetController = null;

    private double targetHeading = 0.0;
    private double rotateSetpoint = 0.0;
    private double prevError = 0.0;
    private int prevXError = 0;

    private Alliance alliance;

    public AutonStateMachine(Alliance a, AutonOp p) {
        this.alliance = a;
        this.parent = p;
    }

    public void setDelay(boolean d) {
        this.delay = d;
    }

    public void setBeacon(Beacon b) {
        this.beacon = b;
    }

    public void setBeaconTargetController(BeaconTargetController bc) {
        this.beaconTargetController = bc;
    }

    public double getTargetHeading() {
        return this.targetHeading;
    }

    public void setRamp(Ramp r) {
        this.ramp = r;
    }

    public void setRampTargetController(RampTargetController rc) {
        this.rampTargetController = rc;
    }

    public void setRobotController(RobotController rc) {
        this.robotController = rc;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public int getState() {
        return this.state;
    }

    public void start() {
        this.state = 0;
    }

    public int getStateCnt() {
        return this.stateCnt;
    }

    private void controlGyro(double leftMotorPower,double rightMotorPower, double gainP, double gainD) {
        double error = targetHeading - robotController.getGyroSensor().getHeading();
        if (error > 180.0)
            error -= 360.0;
        else if (error < -180.0)
            error += 360.0;
        double deltaError = error - prevError;

        double adj = Range.clip(gainP * error + gainD * deltaError, -0.4, 0.4);
        prevError = error;
        fileLogger.writeEvent("auton.control()", "st:"+state+" cnt:"+stateCnt+" tgtHe: "+targetHeading+"heading: "+robotController.getGyroSensor().getHeading()+" err: "+error+" beacSt: " + beaconTargetController.getState() + " bw: " + beacon.getWidth() + " tgtX: " + beaconTargetController.getTargetX() + " beacIllSt: " + beacon.getIlluminationState() + " beacCnt: " + beaconTargetController.getBeaconCount());
        robotController.setLeftDriveMotorPowerTarget(leftMotorPower+adj);
        robotController.setRightDriveMotorPowerTarget(rightMotorPower-adj);
    }

    private void controlX(int xError, double leftMotorPower,double rightMotorPower, double gainP, double gainD) {
        // Drive towards Beacon, while tracking
        int deltaXError = xError - prevXError;
        double adj = Range.clip(gainP * xError + gainD * deltaXError, -0.4, 0.4);
        prevXError = xError;
        fileLogger.writeEvent("auton.control()", "st: " + state + " cnt: " + stateCnt + " x: " + beacon.getX() + " err: " + xError + " beacSt: " + beaconTargetController.getState() + " bw: " + beacon.getWidth() + " tgtX: " + beaconTargetController.getTargetX() + " beacIllSt: " + beacon.getIlluminationState() + " beacCnt: " + beaconTargetController.getBeaconCount());
        double pwr = leftMotorPower-adj;
        robotController.setLeftDriveMotorPowerTarget(pwr); //  -adj 0.2
        pwr = rightMotorPower+adj;
        robotController.setRightDriveMotorPowerTarget(pwr);
    }

    public void process() {

        int nextState = state;

        switch(state) {
            case 0: {
                // INIT
                robotController.setArmAngleServoPos(RobotController.GRABBER_ARM_ANGLE_UP);
// C1                robotController.setRightGrabberServoPos(RobotController.RIGHT_GRABBER_OPEN);
                robotController.setRightGrabberServoPos(RobotController.RIGHT_GRABBER_CLOSED);

                fileLogger.writeEvent("auton.process()", "state: 0");
                if (alliance.isBlueAlliance())
                    beacon.setX(450); // 350 TODO - Set to right or left  If Blue - further to left
                else
                    beacon.setX(75);  // 130 If red, further to right

                if (this.delay) {
                    if (stateCnt > 300)
                        nextState = 1;

                } else {
                    if ((stateCnt > 30) && (!robotController.getGyroSensor().isCalibrating())) //30
                        nextState = 1;
                }
                break;
            }
            case 1: {
                fileLogger.writeEvent("auton.process()", "state: " + state + " cnt: " + stateCnt);
                robotController.setLeftDriveMotorPowerTarget(0.6); // 0.6
                robotController.setRightDriveMotorPowerTarget(0.6); // 0.6
                this.targetHeading = 0.0;

                robotController.setArmSliderServoPower(0.49);
                if (alliance.isBlueAlliance()) {
                    robotController.setArmRotatePower(-0.3); // -.4
                }
                else {
                    robotController.setArmRotatePower(0.3); // .4
                }

                if (stateCnt > 12) { // C2 15  wait for arm up for vision targeting
                    nextState = 10; // 2
                    robotController.setArmRotatePower(0.0);
                    robotController.setArmSliderServoPower(0.0);
                    OpenCVCameraViewListener.getInstance().setState(OpenCVCameraViewListener.INIT_STATE);
                    beaconTargetController.setState(BeaconTargetController.INIT_STATE); // reset beacon left/right detector
                }
                break;
            }
            case 10: {
                controlGyro(0.6,0.6,0.2, 0.001); // 0.02

                if (stateCnt > 100) {
                    if (alliance.isBlueAlliance()) {
                        if (beacon.getX() > 370 && !beacon.isBeaconAcquired())  // 410
                            nextState = 11;
                    } else {
                        if (beacon.getX() < 120 && !beacon.isBeaconAcquired()) // 80
                            nextState = 11;
                    }
                }

                if (stateCnt > 210) // 240
                    nextState = 11;
                break;
            }
            case 11: {
                controlGyro(0.6,0.6,0.2, 0.001); // 0.02

                if (stateCnt > 60) //48
                    robotController.setLeftDriveMotorPowerTarget(0.0);
                    robotController.setRightDriveMotorPowerTarget(0.0);
                    nextState = 12;
                break;
            }
            case 12: {
                beaconTargetController.setBeaconArea(new Rect(0,100,479,70));
                if (alliance.isBlueAlliance())
                    beacon.setX(480);
                else
                    beacon.setX(0);
                nextState = 13;
                break;
            }
            case 13: {
                if (alliance.isBlueAlliance()) {
                    if (targetHeading < 45.0)
                        targetHeading += 0.4;//.2
                } else {
                    if (targetHeading > -45.0)
                        targetHeading -= 0.4; //.2
                }

                controlGyro(0.0, 0.0, 0.06, 0.001); // 0.0, 0.0, 0.08

                if (stateCnt > 30  && beacon.isBeaconAcquired()) {
                    if (alliance.isBlueAlliance()) {
                        if (beacon.getX() < 260 && beacon.getPrevX() > beacon.getX()) { // 260
                            targetHeading = robotController.getGyroSensor().getHeading()-2.0;  // -12  adjustment w/ phone position
                            fileLogger.writeEvent("auton.process()", "blue state to 14 target heading " + targetHeading);
                            nextState = 14;
                        }
                    } else {
                        if ((beacon.getX() > 220) && (beacon.getPrevX() < beacon.getX())) // 220
                        {
                            targetHeading = robotController.getGyroSensor().getHeading()+2.0;  // +5
                            fileLogger.writeEvent("auton.process()", "red state to 14 target heading " + targetHeading);
                            nextState = 14;
//                            robotController.raiseLatch();
                        }
                    }
                }

                if (stateCnt > 150) { // 200
                    targetHeading = robotController.getGyroSensor().getHeading();
                    nextState = 14;
                }
                break;
            }
            case 14: {

                int centerX = 0;
                if (alliance.isBlueAlliance())
                    centerX = 230; // 260
                else
                    centerX = 190; // 220

                int beaconCenterError = centerX - beacon.getX();

                controlX(beaconCenterError,0.5,0.5,0.06, 0.001);

                if (stateCnt > 40 && beacon.getWidth() > 200) // 100, 180
                    nextState = 15;

                if (stateCnt > 180) //200
                    nextState = 15;
                break;
            }
            case 15: {
                if (alliance.isBlueAlliance()) {
                    robotController.setLeftDriveMotorPowerTarget(0.5); // C3 0.8
                    robotController.setRightDriveMotorPowerTarget(0.3); // C3 0.4
                } else {
                    robotController.setLeftDriveMotorPowerTarget(0.3);
                    robotController.setRightDriveMotorPowerTarget(0.5);
                }

                if (stateCnt > 20) // C3 15
                    nextState = 20;
                break;
            }
            case 20: {
                // Unload zipliners
                robotController.setLeftDriveMotorPowerTarget(0.0);
                robotController.setRightDriveMotorPowerTarget(0.0);

                fileLogger.writeEvent("auton.process()", "state = 20  press button " + beaconTargetController.getTargetButton());

                // right arm for blue
                if (alliance.isBlueAlliance()) {
                    robotController.setRightAllClearArmServoPower(-0.3);
                }
                else {
                    robotController.setLeftAllClearArmServoPower(0.3); // fwd
                }

                nextState = 21;
                break;
            }
            case 21: { // wait unload zipliners
                fileLogger.writeEvent("auton.process()","state = 21, return arms");
                if (stateCnt > 80) { // 120
                    if (alliance.isBlueAlliance()) {
                        robotController.setRightAllClearArmServoPower(0.1); // return
                    }
                    else {
                        robotController.setLeftAllClearArmServoPower(-0.1); // return
                    }
                    nextState = 25;
                }
                break;
            }
            case 25: {
                    if (!delay)
                        nextState = 30;
                    else
                        nextState = 1010;
                break;
            }

            case 30: {
                // backup
                fileLogger.writeEvent("auton.process()", "state = 30");
                robotController.setLeftDriveMotorPowerTarget(-0.4);
                robotController.setRightDriveMotorPowerTarget(-0.4);
                if (stateCnt > 8) {
                    if (alliance.isBlueAlliance())
                        rotateSetpoint = robotController.getGyroSensor().getHeading() - 140.0; // 120
                    else
                        rotateSetpoint = robotController.getGyroSensor().getHeading() + 140.0; // 120
                    targetHeading = robotController.getGyroSensor().getHeading();
                    nextState = 31;
                }
                break;
            }
            case 31: {
                if (targetHeading < rotateSetpoint)
                    targetHeading += 1.6;//1.4
                else if (targetHeading > rotateSetpoint)
                    targetHeading -= 1.6; //1.4

                controlGyro(0.0, 0.0, 0.065, 0.005); // 0.0, 0.0, 0.065, 0.001

                if (alliance.isBlueAlliance()) {
                    if (robotController.getGyroSensor().getHeading() < rotateSetpoint) {
                        nextState = 32;
                    }
                } else {
                    if (robotController.getGyroSensor().getHeading() > rotateSetpoint) {
                        nextState = 32;
                    }
                }

                if (stateCnt > 140) { // 100
                    nextState = 32; // 32
                }
                break;
            }
            case 32: {
                // backup
                robotController.setLeftDriveMotorPowerTarget(-0.4);
                robotController.setRightDriveMotorPowerTarget(-0.4);
                fileLogger.writeEvent("auton.process()", "state = 32");
                if (stateCnt > 30) { // C4 80
                    nextState = 1000;
                }
                break;
            }
            case 1000: {
                robotController.setArmSliderServoPower(0.49);
                robotController.setLeftDriveMotorPowerTarget(0.0);
                robotController.setRightDriveMotorPowerTarget(0.0);

                if (stateCnt > 55) { // 50
                    robotController.setArmSliderServoPower(0.0);
                    robotController.setArmAngleServoPos(RobotController.GRABBER_ARM_ANGLE_DOWN);
                    nextState = 1001;
                }
                break;
            }
            case 1001: {
                if (alliance.isBlueAlliance()) {
                    robotController.setArmRotatePower(0.3); // .4
                }
                else {
                    robotController.setArmRotatePower(-0.3); // -.4
                }

                if (stateCnt > 12) { // 12
                    robotController.setArmRotatePower(0.0);
                    robotController.setRightGrabberServoPos(RobotController.RIGHT_GRABBER_OPEN);
                    nextState = 1010;
                }
                break;
            }
            case 1010: {
                OpenCVCameraViewListener.getInstance().setState(OpenCVCameraViewListener.STORE_IMAGES_STATE);
                robotController.setLeftDriveMotorPowerTarget(0.0);
                robotController.setRightDriveMotorPowerTarget(0.0);
                fileLogger.writeEvent("auton.process()", "state: "+state);
                nextState = 1011;
                break;
            }
            case 1011: {
                fileLogger.writeEvent("auton.process()", "state: "+state);
                if (OpenCVCameraViewListener.getInstance().getState() == OpenCVCameraViewListener.DONE)
                    nextState = 1012;
                break;
            }
            case 1012: {
                fileLogger.writeEvent("auton.process()", "state: "+state);
                break;
            }
        }


        if (nextState != state) {
            state = nextState;
            stateCnt = 0;
        } else {
            stateCnt++;
        }
    }
}
