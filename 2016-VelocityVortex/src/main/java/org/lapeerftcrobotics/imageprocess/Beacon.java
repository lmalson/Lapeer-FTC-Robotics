package org.lapeerftcrobotics.imageprocess;

import org.lapeerftcrobotics.log.FileLogger;
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
    public final static int ILLUMINATION_ALL_BLUE = 2;
    public final static int ILLUMINATION_ALL_RED = 3;

    private int x = 240; // blue
    private int y = 130;
    private int prevX = 240;
    private int width = 0;
    private int illuminationState = ILLUMINATION_UKNOWN;
    private int blueSizeThreshold = 30; //20
    private int redSizeThreshold = 30;
    int redCnt = 0;
    int blueCnt = 0;

    private boolean beaconAcquired = false;
    private FileLogger fileLogger;

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

        illuminationState = ILLUMINATION_UKNOWN;

        redCnt = 0;
        blueCnt = 0;

        int minError = 1000;
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
                        int diffError = Math.abs(rBlue[j].width-rRed[i].width);
//                        fileLogger.write("Red["+i+"] w: "+rRed[i].width+" Blue["+j+"] w: "+rBlue[j].width+" Dist: "+distance+" Diff: "+diffError);
                        if (diffError < minError && distance < (0.9*(rBlue[j].width+rRed[i].width))) { // search to find the closest comparable rectangles
  //                          fileLogger.write("Red["+i+"] Blue["+j+"] acquired");
                            minError = diffError;
                            beaconAcquired = true;
                            redIndex = i;
                            blueIndex = j;
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
            y = (rBlue[blueIndex].y + (rBlue[blueIndex].height/2) + rRed[redIndex].y + (rRed[redIndex].height/2))/2+yOffset;
        }
        else {   // Check for All Red or All Blue

            int redXSum = 0;
            int redYSum = 0;

            int blueXSum = 0;
            int blueYSum = 0;


            for (int i = 0; i < rRed.length; i++) {
                if (rRed[i] != null && rRed[i].width > redSizeThreshold && rRed[i].height > redSizeThreshold) {
                    redXSum += rRed[i].x + rRed[i].width / 2;
                    redYSum += rRed[i].y + rRed[i].height / 2;
                    redCnt++;
                }
            }

            for (int i = 0; i < rBlue.length; i++) {
                if (rBlue[i] != null && rBlue[i].width > blueSizeThreshold && rBlue[i].height > blueSizeThreshold) {
                    blueXSum += rBlue[i].x + rBlue[i].width / 2;
                    blueYSum += rBlue[i].y + rBlue[i].height / 2;
                    blueCnt++;
                }
            }

            if (blueCnt > 0) {
                prevX = x;
                x = blueXSum / blueCnt + xOffset;
                y = blueYSum / blueCnt + yOffset;
                illuminationState = ILLUMINATION_ALL_BLUE;
            }
            else if (redCnt > 0) {
                prevX = x;
                x = redXSum / redCnt + xOffset;
                y = redYSum / redCnt + yOffset;
                illuminationState = ILLUMINATION_ALL_RED;
            }
            else
                illuminationState = ILLUMINATION_UKNOWN;
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

    public boolean isBlueOnLeft() {
        if (illuminationState == ILLUMINATION_LEFT_BLUE_RIGHT_RED)
            return true;
        else
            return false;
    }

    public boolean isBlueOnRight() {
        if (illuminationState == ILLUMINATION_LEFT_RED_RIGHT_BLUE)
            return true;
        else
            return false;
    }

    public boolean isAllBlue() {
        if (illuminationState == ILLUMINATION_ALL_BLUE)
            return true;
        else
            return false;
    }

    public boolean isAllRed() {
        if (illuminationState == ILLUMINATION_ALL_RED)
            return true;
        else
            return false;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public void reset() {
        int x = 240;
        int y = 130;
        prevX = 0;
        width = 0;
        illuminationState = ILLUMINATION_UKNOWN;
    }
}
