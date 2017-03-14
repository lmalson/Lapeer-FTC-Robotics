package org.lapeerftcrobotics.control;

import org.lapeerftcrobotics.auton.Beacon;
import org.opencv.core.Rect;

/**
 * Created by t6810sm on 11/21/2015.
 *
 * Calculate target
 */
public class BeaconTargetController {

    public final static int INIT_STATE = 0;
    public final static int APPROACH_STATE = 1;
    public final static int APPROACH_STATE2 = 2;
    public final static int ROTATE_TO_BUTTON_STATE = 3;
    public final static int TO_BUTTON_STATE = 4;

    public final static int TARGET_UNKOWN_BUTTON = -1;
    public final static int TARGET_LEFT_BUTTON = 0;
    public final static int TARGET_RIGHT_BUTTON = 1;

    private int targetButtonSum = 0;
    private int targetButton = TARGET_UNKOWN_BUTTON;
    private final static int CAMERA_CENTER_PX = 240;
    private int state = INIT_STATE;
    private int stateCnt = 0;
    private int targetX = 400;
    private int error = 0;
    private Alliance alliance = null;
    private Beacon beacon = null;

    private int beaconCount = 0;

    private Rect beaconArea = new Rect(250,125,175,50);

    public BeaconTargetController(Alliance a) {
        this.alliance = a;
    }

    public void setState(int s) {
        this.state = s;
    }

    public int getState() {
        return this.state;
    }

    public void setBeacon(Beacon b) {
        this.beacon = b;
    }

    public int getTargetButton() {
        return this.targetButton;
    }

    public Rect getBeaconArea() {
        return this.beaconArea;
    }

    public void setBeaconArea( Rect r) {
        this.beaconArea = r;
    }

    public int getBeaconCount() {
        return this.beaconCount;
    }

    /**
     *
     */
    public void process() {

        int nextState = state;

        switch(state) {
            case INIT_STATE: {
                targetButton = TARGET_UNKOWN_BUTTON;
                nextState = APPROACH_STATE;
                break;
            }
            case APPROACH_STATE: {
                // set target to center
                if (alliance.isBlueAlliance()) {
                    targetX = CAMERA_CENTER_PX + 140;
//                    beaconArea = new Rect(239,110,240,60);
                    beaconArea = new Rect(239,110,200,60);
                }
                else {
                    targetX = CAMERA_CENTER_PX - 140;
//                    beaconArea = new Rect(0,110,240,60);
                    beaconArea = new Rect(40,110,200,60);
                }

                // determine target button
                if ((targetButton == TARGET_UNKOWN_BUTTON) && (beacon.isBeaconAcquired()) && (beacon.getIlluminationState() != Beacon.ILLUMINATION_UKNOWN)) {
                    if ((alliance.isBlueAlliance() && (beacon.getIlluminationState() == Beacon.ILLUMINATION_LEFT_BLUE_RIGHT_RED)) ||
                            (alliance.isRedAlliance() && (beacon.getIlluminationState() == Beacon.ILLUMINATION_LEFT_RED_RIGHT_BLUE)))   {
                        // target left button
                        beaconCount--;
                    }
                    else {
                        beaconCount++;
                    }

                    if (beaconCount > 4) {
                        targetButton = TARGET_RIGHT_BUTTON;
                        nextState = APPROACH_STATE2;
                    }
                    else if (beaconCount < -4) {
                        targetButton = TARGET_LEFT_BUTTON;
                        nextState = APPROACH_STATE2;
                    }
                }

                break;
            }
            case APPROACH_STATE2: {
                // set target to center
                    if (alliance.isBlueAlliance())
                        targetX = CAMERA_CENTER_PX + 140;
                    else
                        targetX = CAMERA_CENTER_PX - 140;

                break;
            }
            case ROTATE_TO_BUTTON_STATE: {
                beaconArea = new Rect(0,100,479,70);
                if (alliance.isBlueAlliance())
                    beacon.setX(480);
                else
                    beacon.setX(0);

                if (targetX < CAMERA_CENTER_PX) {
                    targetX += 5;
                    if ( targetX > CAMERA_CENTER_PX) {
                        targetX = CAMERA_CENTER_PX;
                        nextState = TO_BUTTON_STATE;
                    }
                } else if (targetX > CAMERA_CENTER_PX) {
                    targetX -= 5;
                    if ( targetX < CAMERA_CENTER_PX) {
                        targetX = CAMERA_CENTER_PX;
                        nextState = TO_BUTTON_STATE;
                    }
                }
                break;
            }
            case TO_BUTTON_STATE: {
                beaconArea = new Rect(40,100,439,150);
                targetX = CAMERA_CENTER_PX;
                break;
            }
        }

        if (nextState != state) {
            state = nextState;
            stateCnt = 0;
        } else {
            stateCnt++;
        }

        if (beacon.isBeaconAcquired())
            error = targetX - beacon.getX();
//        {
//        } else {
//            if (alliance.isBlueAlliance())
//                error = -200;
//            else
//                error = 200;
//        }

    }

    /**
     * Pos error + = need to rotate clockwise (right)
     * - = need to rotate counterclockwise (left)
     * @return error
     */
    public int getError() {
        return this.error;
    }

    public int getTargetX() {
        return this.targetX;
    }

}




/**

 if (beacon.getWidth() < 60) {
 targetX = CAMERA_CENTER_PX + 90 + beacon.getWidth();
 } else if (beacon.getWidth() < 150) {
 targetX = CAMERA_CENTER_PX + (int)(5.0/3.0)*(150-beacon.getWidth());
 } else {
 targetX = CAMERA_CENTER_PX;
 }
 else { // RED
 if (beacon.getWidth() < 60) {
 targetX = CAMERA_CENTER_PX - 90 - beacon.getWidth();
 } else if (beacon.getWidth() < 150) {
 targetX = CAMERA_CENTER_PX - (int)(5.0/3.0)*(150-beacon.getWidth());
 } else {
 targetX = CAMERA_CENTER_PX;
 }
 }
*/