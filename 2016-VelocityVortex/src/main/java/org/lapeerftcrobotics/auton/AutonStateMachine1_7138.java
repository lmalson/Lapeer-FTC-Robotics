package org.lapeerftcrobotics.auton;

import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.imageprocess.Beacon;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;

/**
 * Created by User on 10/14/2016.
 */
public class AutonStateMachine1_7138 extends AutonBaseImpl {

    public final static int BEACON_CENTER_ADJ = -24;
    private final static double BEACON_KP = 0.06; // .05
    private double beaconHeadingAdj = 0.0;
    private int lastFrameCnt = -1;
    private int blueLeftCnt = 0;
    private int blueRightCnt = 0;

    private boolean first = true;

    private boolean acquiredBeaconDuringTurn = false;
    private boolean leftButton = false;
    private boolean rightButton = false;

    public AutonStateMachine1_7138() {
    }

    @Override
    public int processStates(int state) {
        int nextState = state;


        telemetry.addData("7138","Auton 1 ");
        telemetry.addData("State", "State: " + state);

        double strikerDistance = robotController.getStrikerDistance();
        double deltaStrikerDistance = robotController.getDeltaStrikerDistance();

        double rightPower = robotController.getRightPower();
        double leftPower = robotController.getLeftPower();
        double heading = robotController.getHeading();
        double driveAngle = robotController.getDriveAngle();
        double headingError = robotController.getHeadingError();
        double distance = robotController.getDistance();
        int frameCnt = imageProcessingManager.getFrameCnt();

        Beacon beacon = imageProcessingManager.getBeacon();
        int beaconX = beacon.getX();
        int beaconWidth = beacon.getWidth();

        if (state > -1) {
            fileLogger.write(""+runtime.milliseconds()+","+state+","+stateCnt+","+strikerDistance+","
                    +deltaStrikerDistance+","+leftPower+","+rightPower+","+heading+","+driveAngle+","+headingError+","
                    +frameCnt+","+beaconHeadingAdj+","+beaconX+","+beaconWidth+","+blueLeftCnt+","+blueRightCnt+","+
                    robotController.getTargetHeading() + "," + robotController.getDistance());
        }

        switch(state) {
            case -1: // Start
                fileLogger.write("time,state,stateCnt,strikerD,strikerDD,leftPwr,rightPwr,gyro,driveAngle,headingError,frameCnt,beaconHeadingAdj,beaconX,beaconWidth,blueLeftCnt,blueRightCnt,targetHeading,distance");
                nextState = 0;
                break;
            case 0:
                robotController.setDriveAngleTarget(0.0);
                robotController.setDriveThrottleTarget(0.0);
                if (!robotController.isGyroCalibrating())
                    nextState = 1;
                break;
            case 1:
                robotController.setIsAutoHeading(true);
                double throt = 0.02 * stateCnt;
                if (throt > 0.4)
                    throt = 0.4;
                robotController.setDriveThrottleTarget(throt);
                if (stateCnt > 17) { // 22 16
                    nextState = 2;
                    robotController.setDriveThrottleTarget(0.0);
                    robotController.setIsAutoHeading(false);
                }
                break;
            case 2: // Fire 1st Ball
                robotController.setIsCannonOn(true);
                if ((deltaStrikerDistance > 0.005 && stateCnt > 15) || stateCnt > 70) { // 2sec.
                    robotController.openHopperDoor();
                    robotController.setIsCannonOn(false);
                    nextState = 10;
                }
                break;
            case 10: // Load 2nd Ball
                if (stateCnt > 10) {  // ~300ms
                    robotController.setIsCannonOn(true);
                }
                if (stateCnt > 17) {  // ~500ms
                    robotController.closeHopperDoor();
                    nextState = 20;
                }
                break;
            case 20: // Fire 2nd Ball
                if (deltaStrikerDistance > 0.005 || stateCnt > 70) { // 2sec.
                    robotController.setIsCannonOn(false);
                    robotController.setDriveThrottleTarget(0.6); // .75 .6
                    if (autonOp.isBlueAlliance())
                        robotController.setDriveAngleTarget(0.3); // .4 blue - right
                    else
                        robotController.setDriveAngleTarget(-0.3); // .4 red - left

                    nextState = 30;
                }
                break;
            case 30: // Drive Fwd - Slightly to right / left
                if (stateCnt > 11) {  // 12
                    robotController.setDriveAngleTarget(0.0);
                    nextState = 40;
                }
                break;
            case 40: // Drive Straight
                if (stateCnt > 13) {  // 14
                    robotController.setDriveThrottleTarget(0.3);  // .2
                    if (autonOp.isBlueAlliance())
                        robotController.setDriveAngleTarget(0.7); // blue - right
                    else
                        robotController.setDriveAngleTarget(-0.7); // red - left
                    nextState = 50;
                }
                break;
            case 50: // Rotate
                if (stateCnt > 8) {
                    robotController.setDriveThrottleTarget(0.5); // .75 .6
                    robotController.setDriveAngleTarget(0.0);
                    nextState = 60;
                }
                break;
            case 60: // Drive Straight
                if (stateCnt > 25) { // 23
                    robotController.setDriveThrottleTarget(0.2); // .2 .1
                    if (autonOp.isBlueAlliance()) {
                        robotController.setDriveAngleTarget(0.7); // .7 .6 blue - right
                        imageProcessingManager.startTrackingBlue();
                    }
                    else {
                        robotController.setDriveAngleTarget(-0.7); // .7 -.6 red - left
                        imageProcessingManager.startTrackingRed();
                    }
                    nextState = 70;
                }
                break;
            case 70: // Rotate - We will see the far beacon 1st so rotate 50 deg.
                if (beacon.isBeaconAcquired()) {
                    beaconHeadingAdj = -(ImageProcessingManager.CAMERA_CENTER + BEACON_CENTER_ADJ - beacon.getX()) * BEACON_KP;
                    robotController.setTargetHeading(heading + beaconHeadingAdj);
                    acquiredBeaconDuringTurn = true;
                }

                if ((autonOp.isRedAlliance() && heading < -50.0) || // 80
                        (autonOp.isBlueAlliance() && heading > 50.0) ||
                        stateCnt > 30) { // 30
                    robotController.setDriveThrottleTarget(0.35);  // .4 .25
                    robotController.setDriveAngleTarget(0.0);

//                    robotController.setTargetHeading(heading);
                    if (!acquiredBeaconDuringTurn) {
                        if (autonOp.isBlueAlliance())
                            robotController.setTargetHeading(90.0);
                        else
                            robotController.setTargetHeading(-90.0);
                    }
                    acquiredBeaconDuringTurn = false;

                    robotController.setIsAutoHeading(true);
                    nextState = 80;
                }
                break;
            case 80: // Heading Control - Use Beacon Targeting

                // calculate heading error
                if (frameCnt != lastFrameCnt) {
                    lastFrameCnt = frameCnt;
                    if (beacon.isBeaconAcquired()) {
                        beaconHeadingAdj = -(ImageProcessingManager.CAMERA_CENTER + BEACON_CENTER_ADJ - beacon.getX())*BEACON_KP;
                        robotController.setTargetHeading(heading + beaconHeadingAdj);
                        if (beacon.isBlueOnRight())
                            blueRightCnt++;
                        else
                            blueLeftCnt++;
                    }
                    else {
                        if (autonOp.isBlueAlliance()) {
                            robotController.setTargetHeading(90.0);
                        }
                        else {
                            robotController.setTargetHeading(-90.0);
                        }
                        beaconHeadingAdj = 0.0;
                    }
                }

                if(distance < 26.0 || beaconWidth > 240){
                    selectButton();
                }

                if (rightButton)
                    robotController.rightButtonOut();
                else if (leftButton)
                    robotController.leftButtonOut();

                if (beaconWidth > 450 || (!beacon.isBeaconAcquired() && stateCnt > 90) || stateCnt > 120 || (distance < 7.0 && stateCnt > 35) ) { // 8
                    robotController.setDriveThrottleTarget(0.0);
                    if (autonOp.isBlueAlliance())
                        robotController.setTargetHeading((90.0 + heading) / 2.0);
                    else
                        robotController.setTargetHeading((-90.0+heading)/2.0);
                    robotController.setIsAutoHeading(true);
                    nextState = 100;
                }
                break;



            case 100: // press button 2nd time
                if (stateCnt > 25) { // 30
                    robotController.setDriveThrottleTarget(-0.5); // .5 .4
                    imageProcessingManager.stopTracking();
                    beacon.reset();
                    resetButtons();
                    nextState = 110;
                }
                break;
            case 110: // backup
                if (stateCnt > 22) { // 28
                    robotController.setIsAutoHeading(false);
                    robotController.setDriveThrottleTarget(0.1); // 0
                    if (autonOp.isBlueAlliance()) {
                        robotController.setDriveAngleTarget(-0.6); // .7 -.6 blue - left
                    }
                    else {
                        robotController.setDriveAngleTarget(0.6); // .7 .6 red - right
                    }
                    nextState = 120;
                }
                break;
            case 120: // turn
                if ((autonOp.isRedAlliance() && heading > -20.0) ||
                        (autonOp.isBlueAlliance() && heading < 20.0) ||
                        stateCnt > 30) { // 30
                    robotController.setDriveThrottleTarget(0.65);  // .6
                    robotController.setDriveAngleTarget(0.0);
                    robotController.setTargetHeading(0.0);
                    robotController.setIsAutoHeading(true);
                    nextState = 130;
                }
                break;
            case 130: // backup - toward 2nd beacon    ADJUST - Distance to 2nd Beacon
                if (stateCnt > 36) { // 42
                    robotController.setDriveThrottleTarget(0.1); // .2 .1
                    robotController.setIsAutoHeading(false);
                    if (autonOp.isBlueAlliance()) {
                        robotController.setDriveAngleTarget(0.55); // .7 .6 blue - right
                        imageProcessingManager.startTrackingBlue();
                    }
                    else {
                        robotController.setDriveAngleTarget(-0.55); // .7 .6 red - left
                        imageProcessingManager.startTrackingRed();
                    }
                nextState = 140;
                }
                break;
            case 140: // Rotate - 2nd beacon
                if (beacon.isBeaconAcquired()) {
                    beaconHeadingAdj = -(ImageProcessingManager.CAMERA_CENTER + BEACON_CENTER_ADJ - beacon.getX()) * BEACON_KP;
                    robotController.setTargetHeading(heading + beaconHeadingAdj);
                    acquiredBeaconDuringTurn = true;
                }

                if ((autonOp.isRedAlliance() && heading < -50.0) || // 80
                        (autonOp.isBlueAlliance() && heading > 50.0) ||
                        stateCnt > 30) { // 30
                    robotController.setDriveThrottleTarget(0.35);  // .4 .25 TODO  speed driving to beacon
                    robotController.setDriveAngleTarget(0.0);

                    if (!acquiredBeaconDuringTurn) {
                        if (autonOp.isBlueAlliance())
                            robotController.setTargetHeading(90.0);
                        else
                            robotController.setTargetHeading(-90.0);
                    }
                    acquiredBeaconDuringTurn = false;

                    robotController.setIsAutoHeading(true);
                    blueRightCnt = 0;
                    blueLeftCnt = 0;
                    nextState = 150;
                }
                break;
            case 150: // Heading Control - Use Beacon Targeting

                // calculate heading error
                if (frameCnt != lastFrameCnt) {
                    lastFrameCnt = frameCnt;
                    if (beacon.isBeaconAcquired()) {
                        beaconHeadingAdj = -(ImageProcessingManager.CAMERA_CENTER + BEACON_CENTER_ADJ - beacon.getX())*BEACON_KP;
                        robotController.setTargetHeading(heading + beaconHeadingAdj);
                        if (beacon.isBlueOnRight())
                            blueRightCnt++;
                        else
                            blueLeftCnt++;
                    }
                    else {
                        if (autonOp.isBlueAlliance()) {
                            robotController.setTargetHeading(90.0);
                        }
                        else {
                            robotController.setTargetHeading(-90.0);
                        }
                        beaconHeadingAdj = 0.0;
                    }
                }

                if(distance < 26.0 || beaconWidth > 240){
                    selectButton();
                }

                if (rightButton)
                    robotController.rightButtonOut();
                else if (leftButton)
                    robotController.leftButtonOut();

                if (beaconWidth > 450 || (!beacon.isBeaconAcquired() && stateCnt > 90) || stateCnt > 120 || distance < 7.0 ) { // 11
                    robotController.setDriveThrottleTarget(0.0);
                    if (autonOp.isBlueAlliance())
                        robotController.setTargetHeading((90.0+heading)/2.0);
                    else
                        robotController.setTargetHeading((-90.0+heading)/2.0);
                    robotController.setIsAutoHeading(true);
                    nextState = 170;
                }
                break;



            case 160:
                if (stateCnt > 8) {
                    robotController.rightButtonIn();
                    robotController.leftButtonIn();
                    nextState = 161;
                }
                break;
            case 161:
                if (stateCnt > 8) {
                    if (rightButton)
                        robotController.rightButtonOut();
                    else if (leftButton)
                        robotController.leftButtonOut();
                    nextState = 162;
                }
                break;
            case 162:
                if (stateCnt > 20) {
                    robotController.rightButtonIn();
                    robotController.leftButtonIn();
                    nextState = 163;
                }
            case 163:
                if (stateCnt > 8) {
                    if (rightButton)
                        robotController.rightButtonOut();
                    else if (leftButton)
                        robotController.leftButtonOut();
                    nextState = 170;
                    robotController.setDriveThrottleTarget(0.0);
                    robotController.setDriveAngleTarget(0.0);
                }
                break;


            case 170: // press button 2nd time
                if (stateCnt > 30) { // 8
                    imageProcessingManager.done();
                    robotController.setDriveThrottleTarget(0.0);  //
                    robotController.setDriveAngleTarget(0.0);
                    robotController.setIsAutoHeading(false);
                    robotController.rightButtonIn();
                    robotController.leftButtonIn();
                    nextState = 180;
                }
                break;

        }

        return nextState;
    }

    private void resetButtons() {
        leftButton = false;
        rightButton = false;
        robotController.rightButtonIn();
        robotController.leftButtonIn();
    }

    private void selectButton() {
        leftButton = false;
        rightButton = false;
        if (autonOp.isBlueAlliance()) {
            if (blueRightCnt > 3)
                leftButton = true;
            else if (blueLeftCnt > 3)
                rightButton = true;
        }
        else {
            if (blueRightCnt > 3)
                rightButton = true;
            else if (blueLeftCnt > 3)
                leftButton = true;
        }
    }

}
