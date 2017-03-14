package org.lapeerftcrobotics.control;

import org.lapeerftcrobotics.auton.Ramp;

/**
 * Created by t6810sm on 11/23/2015.
 */
public class RampTargetController {

    public final static int INIT_STATE = 0;
    public final static int APPROACH_STATE = 1;

    private final static int CAMERA_CENTER_PX = 240;
    private int state = INIT_STATE;
    private int targetX = 0;
    private int error = 0;
    private Alliance alliance = null;
    private Ramp ramp = null;

    public RampTargetController(Alliance a) {
        this.alliance = a;
        if (alliance.isBlueAlliance())
            targetX = CAMERA_CENTER_PX + 90;
        else
            targetX = CAMERA_CENTER_PX - 90;
    }

    public void setState(int s) {
        this.state = s;
    }

    public void setRamp(Ramp r) {
        this.ramp = r;
    }

    public void process() {

        switch (state) {
            case INIT_STATE: {
                state = APPROACH_STATE;
                break;
            }
            case APPROACH_STATE: {

            }
        }
    }
}
