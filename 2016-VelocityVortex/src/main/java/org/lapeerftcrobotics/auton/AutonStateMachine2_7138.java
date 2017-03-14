package org.lapeerftcrobotics.auton;

import org.lapeerftcrobotics.imageprocess.Beacon;

/**
 * Created by User on 10/14/2016.
 */
public class AutonStateMachine2_7138 extends AutonBaseImpl {

    private final static double BEACON_KP = 0.06; // .05
    private double beaconHeadingAdj = 0.0;
    private int lastFrameCnt = -1;
    private int blueLeftCnt = 0;
    private int blueRightCnt = 0;

    private boolean first = true;

    private boolean acquiredBeaconDuringTurn = false;
    private boolean leftButton = false;
    private boolean rightButton = false;

    public AutonStateMachine2_7138() {
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
                if (!robotController.getGyroSensor().isCalibrating())
                    nextState = 1;
                break;
            case 1:
                robotController.setIsAutoHeading(true);
                double throt = 0.02 * stateCnt;
                if (throt > 0.4)
                    throt = 0.4;
                robotController.setDriveThrottleTarget(throt);
                if (stateCnt > 22) { // 16
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
                    robotController.setDriveThrottleTarget(0.0);

                    nextState = 100;
                }
                break;
            case 100:
                break;
        }

        return nextState;
    }

    private void resetButtons() {
        leftButton = false;
        rightButton = false;
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
