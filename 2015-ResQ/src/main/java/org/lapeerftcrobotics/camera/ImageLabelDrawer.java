package org.lapeerftcrobotics.camera;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachine;
import org.lapeerftcrobotics.auton.Beacon;
import org.lapeerftcrobotics.control.Alliance;
import org.lapeerftcrobotics.control.BeaconTargetController;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.opmodes.AutonOp;
import org.lapeerftcrobotics.sensor.SensorListener;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

/**
 * Created by t6810sm on 11/24/2015.
 */
public class ImageLabelDrawer {

    private final static Scalar CONTOUR_COLOR_RED_RGB = new Scalar(255,0,0,255);
    private final static Scalar CONTOUR_COLOR_GREEN_RGB = new Scalar(0,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLUE_RGB = new Scalar(0,0,255,255);
    private final static Scalar CONTOUR_COLOR_WHITE_RGB = new Scalar(255,255,255,255);

    private String version = "";
    private Scalar hsv = null;
    private long frameDuration = 0;
    private Alliance alliance = null;
    private Beacon beacon = null;
    private BeaconTargetController beaconTargetController = null;
    private AutonStateMachine autonStateMachine = null;
    private RobotController robotController = null;

    private int camState = 0;
    private ElapsedTime elapsedTime;

    public void ImageLabelDrawer() {

    }

    public void setElapsedTime(ElapsedTime e) {
        this.elapsedTime = e;
    }

    public void setHsv(Scalar s) {
        this.hsv = s;
    }

    public void setFrameDuration(long d) {
        this.frameDuration = d;
    }

    public void setVersion(String v) {
        this.version = v;
    }

    public void setAlliance(Alliance a) {
        this.alliance = a;
    }

    public void setBeaconTargetController(BeaconTargetController btc) {
        this.beaconTargetController = btc;
    }

    public void setAutonStateMachine(AutonStateMachine a) {
        this.autonStateMachine = a;
    }

    public void setCameraState(int s) {
        this.camState = s;
    }

    public void setBeacon(Beacon b) {
        this.beacon = b;
    }

    public void setRobotController(RobotController rc) {
        this.robotController = rc;
    }

    public void draw(Mat mRgba) {

        Core.rectangle(mRgba, new Point(0, 260), new Point(480, 320), new Scalar(0, 0, 0, 0), -1);

        // Col 1
        Core.putText(mRgba,"Ver:"+ AutonOp.AUTON_VERSION, new Point(10, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba,"Tm:" + String.format("%.3f",elapsedTime.time()), new Point(10, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        String atxt = "N/A";
        if (alliance != null) {
            if (alliance.isBlueAlliance())
                atxt = "BLUE";
            else
                atxt = "RED";
        }
        Core.putText(mRgba,"A:" +atxt , new Point(10, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        // Col 2
        Core.putText(mRgba,"Dur:" + frameDuration, new Point(80, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
//        Core.putText(mRgba,"H:"+(int)hsv.val[0], new Point(80, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
//        Core.putText(mRgba,"S:"+(int)hsv.val[1], new Point(80, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
//        Core.putText(mRgba,"V:"+(int)hsv.val[2], new Point(80, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        // Col 3
        SensorListener sensorListener = SensorListener.getInstance();
        // A - Rotation
        Core.putText(mRgba,"a:"+String.format("%.1f",sensorListener.getAzimuth()), new Point(140, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba,"p:"+String.format("%.1f",sensorListener.getPitch()), new Point(140, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba,"r:"+String.format("%.1f",sensorListener.getRoll()), new Point(140, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        // Col 4
        Core.putText(mRgba,"CSt:"+camState, new Point(210, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        if (autonStateMachine != null)
            Core.putText(mRgba,"ASt:"+autonStateMachine.getState(), new Point(210, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        String txt = "";
        if (beaconTargetController.getTargetButton() == BeaconTargetController.TARGET_LEFT_BUTTON) {
            txt = "LEFT";
        } else {
            txt = "RIGHT";
        }
        Core.putText(mRgba,"TB:" +txt, new Point(210, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);


        // Col 5
        Core.putText(mRgba, "B(X):"+beacon.getX(), new Point(300, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba, "B(W):  "+beacon.getWidth(), new Point(300, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba, "BTgtX:" +beaconTargetController.getTargetX(), new Point(300, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        // Col 6
        Core.putText(mRgba, "TH:" +String.format("%.2f",autonStateMachine.getTargetHeading()), new Point(400, 275), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba, "H:" + robotController.getGyroSensor().getHeading(), new Point(400, 290), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Core.putText(mRgba, "AStC:" + autonStateMachine.getStateCnt(), new Point(400, 305), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        int x = beaconTargetController.getTargetX();
        Core.arrowedLine(mRgba,new Point(x,260),new Point(x,275),CONTOUR_COLOR_GREEN_RGB);
        Core.arrowedLine(mRgba,new Point(beacon.getX(),275),new Point(beacon.getX(),260),CONTOUR_COLOR_WHITE_RGB);

    }
}
