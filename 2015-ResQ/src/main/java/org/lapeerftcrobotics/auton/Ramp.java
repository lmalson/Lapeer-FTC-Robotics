package org.lapeerftcrobotics.auton;

import org.opencv.core.Rect;

/**
 * Created by t6810sm on 11/23/2015.
 */
public class Ramp {

    public final static int RED = 0;
    public final static int BLUE = 1;

    private int color;
    private int x = 0;
    private int y = 0;
    private Rect rBounds = new Rect(0,0,0,0);

    public Ramp() {

    }

    public void process() {

    }

    public void setColor(int c) {
        this.color = c;
    }

    public int getX() {
        return this.x;
    }

    public int getY() {
        return this.y;
    }

    public Rect getBoundingRect() {
        return this.rBounds;
    }

    public int getWidth() {
        return this.rBounds.width;
    }

}
