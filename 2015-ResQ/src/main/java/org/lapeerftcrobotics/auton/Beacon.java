package org.lapeerftcrobotics.auton;

import org.opencv.core.Rect;

/**
 * Created by t6810sm on 11/21/2015.
 *
 *
 */
public class Beacon {

    public final static int ILLUMINATION_UKNOWN = -1;
    public final static int ILLUMINATION_LEFT_BLUE_RIGHT_RED = 0;
    public final static int ILLUMINATION_LEFT_RED_RIGHT_BLUE = 1;

    private int x = 240; // blue
    private int y = 130;
    private int prevX = 240;
    private int width = 40;
    private int illuminationState = ILLUMINATION_UKNOWN;
    private int blueSizeThreshold = 12;
    private int redSizeThreshold = 12;

    private boolean beaconAcquired = false;

    public Beacon() {
    }

    public boolean isBeaconAcquired() {
        return beaconAcquired;
    }

    public void setX(int x) {
        this.x = x;
        this.prevX = x;
    }

    public void processLights(Rect[] rBlue, Rect[] rRed, int xOffset, int yOffset) {
        beaconAcquired = false;

        int dist = 100000;
        int blueIndex = 0;
        int redIndex = 0;

            for(int i=0; i<rRed.length; i++) {
                for(int j=0; j<rBlue.length; j++) {
                    if (rBlue[j] != null && rRed[i] != null && rBlue[j].width > blueSizeThreshold && rBlue[j].height > blueSizeThreshold && rRed[i].width > redSizeThreshold && rRed[i].height > redSizeThreshold) {
                        int xBlue = rBlue[j].x + rBlue[j].width/2;
                        int xRed = rRed[i].x + rRed[i].width/2;
                        int yBlue = rBlue[j].y + rBlue[j].height/2;
                        int yRed = rRed[i].y + rRed[i].height/2;
                        int distance = (int)Math.sqrt((double)((xBlue-xRed)*(xBlue-xRed)+(yBlue-yRed)*(yBlue-yRed)));
                        if (distance < dist && distance < width) {
                            beaconAcquired = true;
                            redIndex = i;
                            blueIndex = j;
                            redSizeThreshold = (int)(Math.min(rRed[i].width,rRed[i].height) * 0.7);
                            blueSizeThreshold = (int)(Math.min(rBlue[j].width,rBlue[j].height) * 0.7);
                        }
                    }
                }
            }

        if (beaconAcquired) {
            int xBlueCenter = rBlue[blueIndex].x + rBlue[blueIndex].width/2;
            int xRedCenter = rRed[redIndex].x + rRed[redIndex].width/2;

            if (xBlueCenter > xRedCenter) {
                // Blue on Right
                illuminationState = ILLUMINATION_LEFT_RED_RIGHT_BLUE;
                width = rBlue[blueIndex].x + rBlue[blueIndex].width - rRed[redIndex].x;
            }
            else {
                // Blue on Left
                illuminationState = ILLUMINATION_LEFT_BLUE_RIGHT_RED;
                width = rRed[redIndex].x + rRed[redIndex].width - rBlue[blueIndex].x;
            }
            prevX = x;
            x = (xBlueCenter+xRedCenter)/2+xOffset;
//            if (Math.abs(xNew - x) < 100) {
//                x = xNew;
//            }
            y = (rBlue[blueIndex].y + (rBlue[blueIndex].height/2) + rRed[redIndex].y + (rRed[redIndex].height/2))/2+yOffset;
        }
    }

    public int getX() { return this.x; }

    public int getY() {
        return this.y;
    }

    public int getPrevX() {return this.prevX;}

    public int getIlluminationState() {
        return this.illuminationState;
    }

    public int getWidth() {
        return this.width;
    }

}
