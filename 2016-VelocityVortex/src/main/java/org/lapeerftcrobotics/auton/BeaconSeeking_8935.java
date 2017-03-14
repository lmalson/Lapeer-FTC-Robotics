package org.lapeerftcrobotics.auton;

import org.lapeerftcrobotics.imageprocess.Beacon;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;

/**
 * Created by User on 10/14/2016.
 */
public class BeaconSeeking_8935 extends AutonBaseImpl {

//    private final static double BEACON_KP = 0.06; // .05
//    private final static double ALIGNMENT_KP = 0.06; // .05
    private double beaconHeadingAdj = 0.0;
    private int lastFrameCnt = -1;
    private int blueLeftCnt = 0;
    private int blueRightCnt = 0;

    private boolean first = true;

    private boolean acquiredBeaconDuringTurn = false;
    private boolean leftButton = false;
    private boolean rightButton = false;

    // TODO
    private double desiredHeading = 0.0;
    private double alignmentError = 0.0;
    private double beaconAlignAdj = 0.0;
    public final static int BEACON_CENTER_ADJ = 45;

    public BeaconSeeking_8935() {
    }

    private double getBeaconTargetHeading(double heading, Beacon beacon) {

        int xError = ImageProcessingManager.CAMERA_CENTER + BEACON_CENTER_ADJ - beacon.getX();
        beaconHeadingAdj = -(xError) * 0.06;    // gyro angle from heading to beacon
        double newTargetHeading = heading + beaconHeadingAdj;
        alignmentError = desiredHeading - newTargetHeading;

        beaconAlignAdj = 0.8 * alignmentError;
        if (beaconAlignAdj > 20.0)
            beaconAlignAdj = 20.0;
        else if (beaconAlignAdj < -20.0)
            beaconAlignAdj = -20.0;

        return newTargetHeading + beaconAlignAdj;
    }

    @Override
    public int processStates(int state) {
        int nextState = state;

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
                    robotController.getTargetHeading() + "," + robotController.getDistance()+","+desiredHeading+","+alignmentError+","+beaconAlignAdj);
        }

        switch(state) {
            case -1: // Start
                fileLogger.write("time,state,stateCnt,strikerD,strikerDD,leftPwr,rightPwr,gyro,driveAngle,headingError,frameCnt,beaconHeadingAdj,beaconX,beaconWidth,blueLeftCnt,blueRightCnt,targetHeading,distance,desiredHeading,alignmentError,beaconAlignAdj");
                nextState = 0;
                break;
            case 0:
                robotController.setDriveAngleTarget(0.0);
                robotController.setDriveThrottleTarget(0.0);
                if (!robotController.isGyroCalibrating())
                    nextState = 60;
                break;
            case 60: // Drive Straight --- adjust toward 1st beacon
                if (stateCnt > 2) {
                    robotController.setDriveThrottleTarget(0.1);

                    if (autonOp.isBlueAlliance()) {
                        robotController.setDriveAngleTarget(0.55); // .6 blue - right
                        imageProcessingManager.startTrackingBlue();
                        desiredHeading = 90.0;
                    }
                    else {
                        robotController.setDriveAngleTarget(-0.55); // -.6 red - left
                        imageProcessingManager.startTrackingRed();
                        desiredHeading = -90.0;
                    }
                    nextState = 70;
                }
                break;
            case 70: // Rotate - We will see the far beacon 1st so rotate 50 deg.
                if (beacon.isBeaconAcquired()) {
                    robotController.setTargetHeading(getBeaconTargetHeading(heading,beacon));
                    acquiredBeaconDuringTurn = true;
                }

                if ((autonOp.isRedAlliance() && heading < -50.0) || // 80
                        (autonOp.isBlueAlliance() && heading > 50.0) ||
                        stateCnt > 30) { // 30
                    robotController.setDriveThrottleTarget(0.25);  // .2
                    robotController.setDriveAngleTarget(0.0);

                    if (!acquiredBeaconDuringTurn)
                            robotController.setTargetHeading(desiredHeading);

                    acquiredBeaconDuringTurn = false;

                    robotController.setIsAutoHeading(true);
                    resetButtons();
                    nextState = 80;
                }
                break;
            case 80: // Heading Control - Use Beacon Targeting

                // calculate heading error
                if (frameCnt != lastFrameCnt) {
                    lastFrameCnt = frameCnt;
                    if (beacon.isBeaconAcquired()) {
                        robotController.setTargetHeading(getBeaconTargetHeading(heading,beacon));
                        if (beacon.isBlueOnRight())
                            blueRightCnt++;
                        else
                            blueLeftCnt++;
                    }
                    else {
                        robotController.setTargetHeading(desiredHeading);
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

                if (beaconWidth > 450 || (!beacon.isBeaconAcquired() && stateCnt > 90) || stateCnt > 120 || distance < 8.0 ) { // 300
                    robotController.setDriveThrottleTarget(0.0);
                    robotController.setTargetHeading((desiredHeading+heading)/2.0);
                    robotController.setIsAutoHeading(true);
                    nextState = 100;
                }
                break;

            case 100: // press button
                if (stateCnt > 15) { // 30
                    robotController.setDriveThrottleTarget(-0.4);
                    imageProcessingManager.stopTracking();
                    beacon.reset();
                    resetButtons();
                    nextState = 110;
                }
                break;
            case 110: // backup
                imageProcessingManager.done();
                robotController.setDriveThrottleTarget(0.0);  //
                robotController.setDriveAngleTarget(0.0);
                robotController.setIsAutoHeading(false);
                robotController.rightButtonIn();
                robotController.leftButtonIn();
                nextState = 120;
                break;

        }

        return nextState;
    }

    /**
     * Button Pressers are actually opposite as running in reverse
     */

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
