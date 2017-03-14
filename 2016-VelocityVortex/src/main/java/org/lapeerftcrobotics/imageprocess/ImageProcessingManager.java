package org.lapeerftcrobotics.imageprocess;

import android.graphics.Bitmap;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonOp;
import org.lapeerftcrobotics.log.FileLogger;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.util.List;

/**
 * Created by User on 10/19/2016.
 */
public class ImageProcessingManager {

    public final static int ZTE = 0;
    public final static int S5 = 1;

    public static int PHONE_TYPE = S5;

    // Phone Specific
    // Galaxy S5
    public static final int CAMERA_CENTER = 320;
    private final static int WIDTH = 640;
    private final static int HEIGHT = 480;
    private final static double MIN_CONTOUR_AREA = 300.0;

    // ZTE
//    public static final int CAMERA_CENTER = 240;
//    private final static int WIDTH = 480;
//    private final static int HEIGHT = 320;
//    private final static double MIN_CONTOUR_AREA = 200.0;


    private final static boolean SAVE_IMAGES = true;


    public final static int NONE_MODE = 0;
    public final static int TARGET_LINE_MODE = 1;
    public final static int TARGET_BEACON_MODE = 2;
    public final static int SAVE = 6;
    public final static int COMPLETE = 7;

    private final static Scalar CONTOUR_COLOR_RED_RGB = new Scalar(255,0,0,255);
    private final static Scalar CONTOUR_COLOR_GREEN_RGB = new Scalar(0,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLUE_RGB = new Scalar(0,0,255,255);
    private final static Scalar CONTOUR_COLOR_WHITE_RGB = new Scalar(255,255,255,255);
    private final static Scalar CONTOUR_COLOR_YELLOW_RGB = new Scalar(255,255,0,255);


    //  UPPER & LOWER BOUNDS
    private static Scalar BRED_LOWER_BOUNDS_HSV = new Scalar(0,100,150);
    private static Scalar BRED_UPPER_BOUNDS_HSV = new Scalar(30,255,255);
    private static Scalar BRED_LOWER_BOUNDS2_HSV = new Scalar(225,100,150);
    private static Scalar BRED_UPPER_BOUNDS2_HSV = new Scalar(255,255,255);

    private static Scalar BBLUE_LOWER_BOUNDS_HSV = new Scalar(145,100,160); // 100,180,220
    private static Scalar BBLUE_UPPER_BOUNDS_HSV = new Scalar(180,255,255);

    private static Scalar WHITE_LOWER_BOUNDS_HSV = new Scalar(0,0,230);
    private static Scalar WHITE_UPPER_BOUNDS_HSV = new Scalar(180,25,255);

    private int mode = NONE_MODE;
    private int xError = 0;
    private int yError = 0;
    private boolean detected = false;

    private Mat[] savedMats = new Mat[300];
    private int imgCnt = 0;
    private int frameCnt = 0;

    private ColorBlobDetector    mDetectorRed;
    private ColorBlobDetector    mDetectorBlue;
    private ColorBlobDetector    mDetectorWhite;

    private Mat mat1 = null;
    private Mat mat2 = null;
    private Mat mat3 = null;
    private Mat mat4 = null;
    private Mat mat5 = null;
    private Mat mat6 = null;
    private Mat mat7 = null;
    private Mat mHierarchy = null;

    private FileLogger fileLogger;
    private Telemetry telemetry;
    private Beacon beacon;

    private boolean targetBlue = false;
    private AutonOp autonOp;

    public ImageProcessingManager() {

        mDetectorRed = new ColorBlobDetector();
        mDetectorRed.setLowerBounds(BRED_LOWER_BOUNDS_HSV);
        mDetectorRed.setUpperBounds(BRED_UPPER_BOUNDS_HSV);
        mDetectorRed.setLowerBounds2(BRED_LOWER_BOUNDS2_HSV);
        mDetectorRed.setUpperBounds2(BRED_UPPER_BOUNDS2_HSV);
        mDetectorRed.setMinContourArea(MIN_CONTOUR_AREA);

        mDetectorBlue = new ColorBlobDetector();
        mDetectorBlue.setLowerBounds(BBLUE_LOWER_BOUNDS_HSV);
        mDetectorBlue.setUpperBounds(BBLUE_UPPER_BOUNDS_HSV);
        mDetectorBlue.setMinContourArea(MIN_CONTOUR_AREA);

        mDetectorWhite = new ColorBlobDetector();
        mDetectorWhite.setLowerBounds(WHITE_LOWER_BOUNDS_HSV);
        mDetectorWhite.setUpperBounds(WHITE_UPPER_BOUNDS_HSV);
        mDetectorWhite.setMinContourArea(MIN_CONTOUR_AREA);

        mat1 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat2 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat3 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat4 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat5 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat6 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat7 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mHierarchy = new Mat();

        beacon = new Beacon();
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
        beacon.setFileLogger(fileLogger);
    }

    public int getImgCnt() {
        return imgCnt;
    }

    public int getFrameCnt() {
        return frameCnt;
    }

    public void setMode(int m) {
        if (this.mode != m) {
            xError = 0;
            yError = 0;
            detected = false;
        }

        this.mode = m;
    }

    public int getxError() {
        return this.xError;
    }

    public int getyError() {
        return this.yError;
    }

    public boolean isDetected() {
        return this.detected;
    }

    public void startTrackingBlue() {
        this.mode = TARGET_BEACON_MODE;
        targetBlue = true;
    }

    public void startTrackingRed() {
        this.mode = TARGET_BEACON_MODE;
        targetBlue = false;
    }

    public void stopTracking() {
        this.mode = NONE_MODE;
    }

    public void done() {
        this.mode = SAVE;
    }

    public Beacon getBeacon() {
        return this.beacon;
    }

    public Mat process(Mat in) {

        try {

        in.convertTo(mat1, CvType.CV_8UC4);
        Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_RGB2HSV_FULL);

        switch(mode) {
            case NONE_MODE:
                break;
            case TARGET_BEACON_MODE:
                telemetry.addData("Image","Target RED/BLUE "+imgCnt);

                mDetectorBlue.process(mat2);
                Rect[] blueBoundings = mDetectorBlue.getBoundingRectangles();
                drawBoundingRect(in,blueBoundings,CONTOUR_COLOR_BLUE_RGB);

                List<MatOfPoint> contoursB = mDetectorBlue.getContours();
                Imgproc.drawContours(in, contoursB, -1, CONTOUR_COLOR_YELLOW_RGB, 1);

                mDetectorRed.process(mat2);
                Rect[] redBoundings = mDetectorRed.getBoundingRectangles();
                drawBoundingRect(in, redBoundings, CONTOUR_COLOR_RED_RGB);

                List<MatOfPoint> contoursR = mDetectorRed.getContours();
                Imgproc.drawContours(in, contoursR, -1, CONTOUR_COLOR_YELLOW_RGB, 1);

                beacon.processLights(blueBoundings, redBoundings, 0, 0);

                // display on screen
                Imgproc.putText(in, "Frame: " + frameCnt, new Point(20, 400), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
                Imgproc.putText(in, "BeacAcq: " + beacon.isBeaconAcquired(), new Point(20, 420), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
                Imgproc.putText(in, "BeacX: " + beacon.getX(), new Point(20, 440), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
                String orientation = "";
                int bs = beacon.getIlluminationState();
                switch(bs) {
                    case Beacon.ILLUMINATION_UKNOWN:
                        orientation = "No";
                        break;
                    case Beacon.ILLUMINATION_ALL_BLUE:
                        orientation = "All Blue";
                        break;
                    case Beacon.ILLUMINATION_ALL_RED:
                        orientation = "All Red";
                        break;
                    case Beacon.ILLUMINATION_LEFT_BLUE_RIGHT_RED:
                        orientation = "Left Blue";
                        break;
                    case Beacon.ILLUMINATION_LEFT_RED_RIGHT_BLUE:
                        orientation = "Right Blue";
                        break;
                }
                Imgproc.putText(in, "BeacSt: " + orientation, new Point(150, 400), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
                Imgproc.putText(in, "BeacW: " + beacon.getWidth(), new Point(150, 420), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
                Imgproc.putText(in, "Time: " + autonOp.getTime().milliseconds(), new Point(150, 440), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

                Imgproc.line(in, new Point(320, 40), new Point(320, 380), CONTOUR_COLOR_GREEN_RGB, 1);

                if (bs != Beacon.ILLUMINATION_UKNOWN) {
                    if (autonOp.isBlueAlliance()) {
                        Imgproc.rectangle(in,new Point(5,5),new Point(635,475), CONTOUR_COLOR_BLUE_RGB, 2);
                    } else {
                        Imgproc.rectangle(in,new Point(5,5),new Point(635,475), CONTOUR_COLOR_RED_RGB, 2);
                    }
                    Imgproc.line(in, new Point(beacon.getX(), 40), new Point(beacon.getX(), 380), CONTOUR_COLOR_YELLOW_RGB, 1);
                }

                if (imgCnt < 300) {
                    frameCnt++;
                    if (SAVE_IMAGES) {
                        if (frameCnt % 2 == 0) {
                            savedMats[imgCnt] = in.clone();
                            imgCnt++;
                        }
                    }
                }

                break;
            case SAVE:
                imgCnt--;
                if (imgCnt >= 0 && SAVE_IMAGES) {
                    storeImage(imgCnt);
                } else
                    mode = COMPLETE;
                break;
            case COMPLETE:
                break;
            }

        } catch(Exception ex) {
            fileLogger.write("Caught Exception: "+ex+"\n"+getStackTrace(ex));
            ex.printStackTrace();
        }

        return in;
    }

    public static String getStackTrace(Throwable aThrowable) {
        Writer result = new StringWriter();
        PrintWriter printWriter = new PrintWriter(result);
        aThrowable.printStackTrace(printWriter);
        return result.toString();
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
                Log.d("ImageProcessingManager", e.getMessage());
            }
            if (fileLogger != null)
                fileLogger.saveBitmap(bmp,"-img"+i+".png");

            long dur = System.currentTimeMillis() - startTime;
            telemetry.addData("Image","wrote img["+i+"] image file in "+dur+"(ms). ");
            Log.d("ImageProcessingManager", "wrote img["+i+"] image file in "+dur+"(ms). ");
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private void drawBoundingRect(Mat mRgb, Rect[] r, Scalar c) {
        for(int i=0; i<r.length; i++) {
            if (r[i] != null) {
                int x = r[i].x;
                int y = r[i].y;
                Point p1 = new Point(x,y);
                Point p2 = new Point(x+r[i].width,y+r[i].height);
                Imgproc.rectangle(mRgb,p1,p2,c,2);
            }
        }
    }

    public void setAutonOp(AutonOp autonOp) {
        this.autonOp = autonOp;
    }
}
