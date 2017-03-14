package org.lapeerftcrobotics.camera;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachine;
import org.lapeerftcrobotics.auton.Beacon;
import org.lapeerftcrobotics.control.Alliance;
import org.lapeerftcrobotics.control.BeaconTargetController;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.log.FileLogger;
import org.lapeerftcrobotics.opmodes.AutonOp;
import org.lapeerftcrobotics.sensor.SensorListener;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.List;


/**
 * Created by t6810sm on 11/11/2015.
 * - change red saturation levels
 */
public class OpenCVCameraViewListener implements CameraBridgeViewBase.CvCameraViewListener2 {

    public static String TAG = OpenCVCameraViewListener.class.getSimpleName();
    private static OpenCVCameraViewListener instance;
    private FileLogger fileLogger;
    private int imgCnt = 0;
    private int frameCnt = 0;

    private static Scalar ORANGE_MIN = new Scalar(5, 50, 50);
    private static Scalar ORANGE_MAX = new Scalar(15, 255, 255);

    private final static Scalar CONTOUR_COLOR_RED_RGB = new Scalar(255,0,0,255);
    private final static Scalar CONTOUR_COLOR_GREEN_RGB = new Scalar(0,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLUE_RGB = new Scalar(0,0,255,255);
    private final static Scalar CONTOUR_COLOR_WHITE_RGB = new Scalar(255,255,255,255);

//  UPPER & LOWER BOUNDS
    private static Scalar BRED_LOWER_BOUNDS_HSV = new Scalar(0,40,205); // 0,50,200  OLD 0,9,180
    private static Scalar BRED_UPPER_BOUNDS_HSV = new Scalar(30,140,305); // 20,120,255 OLD 20,130,255
    private static Scalar BRED_LOWER_BOUNDS2_HSV = new Scalar(220,0,205);
    private static Scalar BRED_UPPER_BOUNDS2_HSV = new Scalar(255,100,305);

    private static Scalar GREEN_LOWER_BOUNDS_HSV = new Scalar(45,45,75);
    private static Scalar GREEN_UPPER_BOUNDS_HSV = new Scalar(95,255,205);

    private static Scalar BBLUE_LOWER_BOUNDS_HSV = new Scalar(100,60,205); // 140,130,210
    private static Scalar BBLUE_UPPER_BOUNDS_HSV = new Scalar(150,290,305); // 160,255,255

    private static Scalar WHITE_LOWER_BOUNDS_HSV = new Scalar(0,0,230);
    private static Scalar WHITE_UPPER_BOUNDS_HSV = new Scalar(180,25,255);

    private static Scalar BBLACK_LOWER_BOUNDS_HSV = new Scalar(0,0,0);
    private static Scalar BBLACK_UPPER_BOUNDS_HSV = new Scalar(180,255,20);

    public final static int NOOP_STATE = -1;
    public final static int INIT_STATE = 0;
    public final static int FIND_BEACON_STATE = 1;
    public final static int TRACK_BEACON_STATE = 2;
    public final static int TRACK_BUTTON_STATE = 3;
    public final static int STORE_IMAGES_STATE = 100;
    public final static int DONE = 101;

    private int state = NOOP_STATE;
    private int nextState = NOOP_STATE;
    private int stateCnt = 0;
    private long frameDuration = 0;

    private boolean initialized = false;
    private Mat[] savedMats = new Mat[300];

    private ImageLabelDrawer labelDrawer = new ImageLabelDrawer();

    private Mat                  mRgba;
    private ColorBlobDetector    mDetectorRed;
    private ColorBlobDetector    mDetectorGreen;
    private ColorBlobDetector    mDetectorBlue;
    private ColorBlobDetector    mDetectorWhite;
    private ColorBlobDetector    mDetectorBlack;

    private Rect rectBeaconRed = new Rect(250,125,175,50);
    private Rect rectBeaconBlue = new Rect(250,125,175,50);
    private Rect rectBeaconBlack = new Rect(0,0,0,0);
    private Rect rectHsv = new Rect(270,140,6,6);

    private Beacon beacon = null;
    private BeaconTargetController beaconTargetController = null;
    private AutonStateMachine autonStateMachine = null;
    private Alliance alliance = null;
    private RobotController robotController = null;

    public static synchronized OpenCVCameraViewListener getInstance() {
        if (instance == null) {
          instance = new OpenCVCameraViewListener();
        }
        return instance;
    }

    public void setRobotController(RobotController rc) {
        labelDrawer.setRobotController(rc);
    }

    public void setElapsedTime(ElapsedTime e) {
        labelDrawer.setElapsedTime(e);
    }

    public void setBeacon( Beacon b) {
        labelDrawer.setBeacon(b);
        this.beacon = b;
    }

    public void setAlliance(Alliance a) {
        labelDrawer.setAlliance(a);
        this.alliance = a;
    }

    public void setBeaconTargetController(BeaconTargetController btc) {
        labelDrawer.setBeaconTargetController(btc);
        this.beaconTargetController = btc;
    }

    public void setAutonStateMachine(AutonStateMachine a) {
        labelDrawer.setAutonStateMachine(a);
        this.autonStateMachine = a;
    }

    public int getState() {
        return this.state;
    }

    public void setState(int s) {
        this.nextState = s;
        this.state = s;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

        mRgba = new Mat(height, width, CvType.CV_8UC4);

        mDetectorRed = new ColorBlobDetector();
        mDetectorRed.setLowerBounds(BRED_LOWER_BOUNDS_HSV);
        mDetectorRed.setUpperBounds(BRED_UPPER_BOUNDS_HSV);
        mDetectorRed.setLowerBounds2(BRED_LOWER_BOUNDS2_HSV);
        mDetectorRed.setUpperBounds2(BRED_UPPER_BOUNDS2_HSV);
        mDetectorRed.setMinContourArea(0.1);
        mDetectorRed.setDrawColor(CONTOUR_COLOR_RED_RGB);
        mDetectorRed.setSubMatRect(rectBeaconRed);

        mDetectorGreen = new ColorBlobDetector();
        mDetectorGreen.setLowerBounds(GREEN_LOWER_BOUNDS_HSV);
        mDetectorGreen.setUpperBounds(GREEN_UPPER_BOUNDS_HSV);
        mDetectorGreen.setMinContourArea(0.1);
        mDetectorGreen.setDrawColor(CONTOUR_COLOR_GREEN_RGB);

        mDetectorBlue = new ColorBlobDetector();
        mDetectorBlue.setLowerBounds(BBLUE_LOWER_BOUNDS_HSV);
        mDetectorBlue.setUpperBounds(BBLUE_UPPER_BOUNDS_HSV);
        mDetectorBlue.setMinContourArea(0.1);
        mDetectorBlue.setDrawColor(CONTOUR_COLOR_BLUE_RGB);
        mDetectorBlue.setSubMatRect(rectBeaconBlue);

        mDetectorWhite = new ColorBlobDetector();
        mDetectorWhite.setLowerBounds(WHITE_LOWER_BOUNDS_HSV);
        mDetectorWhite.setUpperBounds(WHITE_UPPER_BOUNDS_HSV);
        mDetectorWhite.setMinContourArea(0.1);
        mDetectorWhite.setDrawColor(CONTOUR_COLOR_GREEN_RGB);

        mDetectorBlack = new ColorBlobDetector();
        mDetectorBlack.setLowerBounds(BBLACK_LOWER_BOUNDS_HSV);
        mDetectorBlack.setUpperBounds(BBLACK_UPPER_BOUNDS_HSV);
        mDetectorBlack.setMinContourArea(0.1);
        mDetectorBlack.setDrawColor(CONTOUR_COLOR_GREEN_RGB);
        mDetectorBlack.setSubMatRect(rectBeaconBlack);

        labelDrawer.setVersion(AutonOp.AUTON_VERSION);

        initialized = true;
    }

    @Override
    public void onCameraViewStopped() {
    }

    /**
     * Scalar(B,G,R,Alpha) - RGB Color 255,0,0 = Blue, 0,255,0 = Green, 0,0,255 = Red
     *
     * @param inputFrame
     * @return
     */
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        try {
//            if (fileLogger != null)
//                fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", "start state: "+state);

            Mat mRgba=inputFrame.rgba();

            if (!initialized || state == NOOP_STATE || autonStateMachine == null)
                return mRgba;

            long startTime = System.currentTimeMillis();

            mDetectorWhite.setSubMatRect(rectHsv);
            mDetectorWhite.process(mRgba);
            labelDrawer.setHsv(mDetectorWhite.getHsvColor());
            labelDrawer.setCameraState(this.state);

            nextState = state;

            switch(state) {
                case NOOP_STATE:
                    break;
                case INIT_STATE:
                {
                    imgCnt = 0;
                    nextState = FIND_BEACON_STATE;
                    break;
                }
                case FIND_BEACON_STATE:
                {
                    mDetectorBlue.setSubMatRect(rectBeaconBlue);
                    mDetectorBlue.process(mRgba, false, false);
                    mDetectorRed.setSubMatRect(rectBeaconRed);
                    mDetectorRed.process(mRgba, true);
                    beacon.processLights(mDetectorBlue.getBoundingRectangles(), mDetectorRed.getBoundingRectangles(), rectBeaconBlue.x, rectBeaconBlue.y);
                    Rect area = beaconTargetController.getBeaconArea();
                    rectBeaconBlue = area;
                    rectBeaconRed = area;
                    Point p1 = new Point(area.x,area.y);
                    Point p2 = new Point(area.x+area.width,area.y+area.height);
                    Core.rectangle(mRgba, p1, p2, CONTOUR_COLOR_WHITE_RGB, 1);
                    labelDrawer.draw(mRgba);

                    if (fileLogger != null)
                        fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", " FIND BEACON state: "+state+" imgCnt: "+imgCnt);
                    if (imgCnt < 300) {
                        frameCnt++;
                        if (frameCnt % 2 == 0) {
                            savedMats[imgCnt] = mRgba.clone();
                            imgCnt++;
                        }
                    }
                    break;
                }
                case STORE_IMAGES_STATE:
                {
                    imgCnt--;
                    if (imgCnt >= 0) {
                        if (fileLogger != null)
                            fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", " Store Image imgCnt: "+imgCnt);
                        storeImage(imgCnt);
                    } else
                        nextState = NOOP_STATE;
                    break;
                }
                case DONE:
                    break;
            }

            if (nextState != state) {
                state = nextState;
                stateCnt = 0;
            } else {
                stateCnt++;
            }

            frameDuration = System.currentTimeMillis() - startTime;

//            if (fileLogger != null)
//                fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", "dur(ms): " + frameDuration + " state: "+state+" imgCnt: "+imgCnt);

        } catch (Exception ex) {
            if (fileLogger != null) {
                fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", "XXXXX Exception: " +ex.toString());
                StringWriter sw = new StringWriter();
                ex.printStackTrace(new PrintWriter(sw));
                String exceptionAsString = sw.toString();
                fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", "XXXXX Trace: " +exceptionAsString);
            }
        }

        return mRgba;
    }

    public void storeImages() {
        for(int i=imgCnt-1; i>-1; i--) {
            storeImage(i);
        }
    }

    public void storeImage(int i) {
        if (fileLogger != null) {
            long startTime = System.currentTimeMillis();

            Bitmap bmp = null;
            try {
                bmp = Bitmap.createBitmap(savedMats[i].cols(), savedMats[i].rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(savedMats[i], bmp);
            } catch (CvException e) {
                Log.d(TAG, e.getMessage());
            }
            if (fileLogger != null)
                fileLogger.saveBitmap(bmp,"-img"+i+".png");

            long dur = System.currentTimeMillis() - startTime;
            if (fileLogger != null)
                fileLogger.writeEvent("OpenCVCamVList.onPreviewFrame()", "wrote img["+i+"] image file in "+dur+"(ms). ");
        }
    }
}
