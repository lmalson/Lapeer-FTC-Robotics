package org.lapeerftcrobotics.control;

/**
 * Created by t6810sm on 11/21/2015.
 */
public class Alliance {

    public final static int BLUE_ALLIANCE = 0;
    public final static int RED_ALLIANCE = 1;
    private int allianceColor = -1;

    public Alliance(int color) {
        this.allianceColor = color;
    }

    public boolean isBlueAlliance() {
        if (this.allianceColor == BLUE_ALLIANCE) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isRedAlliance() {
        if (this.allianceColor == RED_ALLIANCE) {
            return true;
        } else {
            return false;
        }

    }

}
