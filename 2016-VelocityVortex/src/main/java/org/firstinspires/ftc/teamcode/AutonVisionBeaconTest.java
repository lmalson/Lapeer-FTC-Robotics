/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.imageprocess.Beacon;
import org.lapeerftcrobotics.imageprocess.ColorBlobDetector;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.lapeerftcrobotics.vision.OnCameraFrameCallbackInterface;
import org.lapeerftcrobotics.vision.OpenCVCameraActivityHelper;
import org.lapeerftcrobotics.vision.OpenCVCameraViewListener;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */

@TeleOp(name="AutonVisionTest", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class AutonVisionBeaconTest extends OpMode implements OnCameraFrameCallbackInterface
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCVCameraActivityHelper openCVCameraActivityHelper = null;

    private final static int WIDTH = 640; // S5    480 ZTE
    private final static int HEIGHT = 480; // S5   320 ZTE

    private static Scalar BRED_LOWER_BOUNDS_HSV = new Scalar(0,100,150);
    private static Scalar BRED_UPPER_BOUNDS_HSV = new Scalar(30,255,255);
    private static Scalar BRED_LOWER_BOUNDS2_HSV = new Scalar(225,100,150);
    private static Scalar BRED_UPPER_BOUNDS2_HSV = new Scalar(255,255,255);

    private static Scalar BBLUE_LOWER_BOUNDS_HSV = new Scalar(145,100,160); // 100,180,220
    private static Scalar BBLUE_UPPER_BOUNDS_HSV = new Scalar(180,255,255);

    private final static Scalar CONTOUR_COLOR_RED_RGB = new Scalar(255,0,0,255);
    private final static Scalar CONTOUR_COLOR_GREEN_RGB = new Scalar(0,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLUE_RGB = new Scalar(0,0,255,255);
    private final static Scalar CONTOUR_COLOR_WHITE_RGB = new Scalar(255,255,255,255);
    private final static Scalar CONTOUR_COLOR_YELLOW_RGB = new Scalar(255,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLACK_RGB = new Scalar(0,0,0,255);

    private ColorBlobDetector mDetectorRed;
    private ColorBlobDetector mDetectorBlue;

    private Beacon beacon;

    private Mat mat1 = null;
    private Mat mat2 = null;
    private Mat mat3 = null;
    private Mat mat4 = null;
    private Mat mat5 = null;
    private Mat mat6 = null;
    private Mat mat7 = null;
    private Mat mHierarchy = null;

    private GyroSensor gyro = null;
    private double desiredHeading = 0.0;
    private double alignmentError = 0.0;
    private double beaconAlignAdj = 0.0;
    private double targetHeading = 0.0;
    private double beaconHeadingAdj = 0.0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        openCVCameraActivityHelper = new OpenCVCameraActivityHelper();
        openCVCameraActivityHelper.init();

        gyro = hardwareMap.gyroSensor.get("gs");
        gyro.calibrate();

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        OpenCVCameraViewListener.getInstance().registerOnFrameCallbackListener(this);
        mat1 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat2 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat3 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat4 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat5 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat6 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mat7 = new Mat(WIDTH,HEIGHT, CvType.CV_8UC4);
        mHierarchy = new Mat();

        mDetectorRed = new ColorBlobDetector();
        mDetectorRed.setLowerBounds(BRED_LOWER_BOUNDS_HSV);
        mDetectorRed.setUpperBounds(BRED_UPPER_BOUNDS_HSV);
        mDetectorRed.setLowerBounds2(BRED_LOWER_BOUNDS2_HSV);
        mDetectorRed.setUpperBounds2(BRED_UPPER_BOUNDS2_HSV);
        mDetectorRed.setMinContourArea(20.0);

        mDetectorBlue = new ColorBlobDetector();
        mDetectorBlue.setLowerBounds(BBLUE_LOWER_BOUNDS_HSV);
        mDetectorBlue.setUpperBounds(BBLUE_UPPER_BOUNDS_HSV);
        mDetectorBlue.setMinContourArea(20.0);

        beacon = new Beacon();

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gyro.isCalibrating()) {
            telemetry.addData("Gyro Calibrating...","");
            return;
        }

        telemetry.addData("Status", "Running: " + runtime.toString());
        int x = beacon.getX();
        int y = beacon.getY();

        int bs = beacon.getIlluminationState();

        switch(bs) {
            case Beacon.ILLUMINATION_UKNOWN:
                telemetry.addData("Beacon","No");
                break;
            case Beacon.ILLUMINATION_ALL_BLUE:
                telemetry.addData("Beacon","ALL BLUE");
                break;
            case Beacon.ILLUMINATION_ALL_RED:
                telemetry.addData("Beacon","ALL RED");
                break;
            case Beacon.ILLUMINATION_LEFT_BLUE_RIGHT_RED:
                telemetry.addData("Beacon","LEFT BLUE");
                break;
            case Beacon.ILLUMINATION_LEFT_RED_RIGHT_BLUE:
                telemetry.addData("Beacon","RIGHT BLUE");
                break;
        }

        targetHeading = getBeaconTargetHeading(gyro.getHeading(),beacon);

        telemetry.addData("BeaconPos","x: "+x+" y: "+y+" w: "+beacon.getWidth());
        telemetry.addData("Heading",""+gyro.getHeading());
        telemetry.addData("TargetHeading",""+targetHeading);
        telemetry.addData("BeaconAcq",""+beacon.isBeaconAcquired());
        telemetry.update();


    }

    private double getBeaconTargetHeading(double heading, Beacon beacon) {

        int xError = ImageProcessingManager.CAMERA_CENTER - beacon.getX();
        double pixelToAngleFactor = 0.06 * (beacon.getWidth() / 500);
        beaconHeadingAdj = -(xError) * pixelToAngleFactor;    // gyro angle from heading to beacon
        double newTargetHeading = heading + beaconHeadingAdj;
        alignmentError = desiredHeading - newTargetHeading;

        beaconAlignAdj = 1.0 * alignmentError;
        if (beaconAlignAdj > 30.0)
            beaconAlignAdj = 30.0;
        else if (beaconAlignAdj < -30.0)
            beaconAlignAdj = -30.0;

        return newTargetHeading; // + beaconAlignAdj;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        openCVCameraActivityHelper.stop();
        OpenCVCameraViewListener.getInstance().registerOnFrameCallbackListener(null);
    }

    /**
     * This method is called by the OpenCV Camera Thread independent of the FTC loop() method.
     * @param in
     * @return
     */

    @Override
    public Mat process(Mat in) {

        in.convertTo(mat1, CvType.CV_8UC4);
        Imgproc.cvtColor(mat1, mat2, Imgproc.COLOR_RGB2HSV_FULL);

        mDetectorBlue.process(mat2);
        Rect[] blueBoundings = mDetectorBlue.getBoundingRectangles();
        drawBoundingRect(in, blueBoundings, CONTOUR_COLOR_BLUE_RGB);

        List<MatOfPoint> contoursB = mDetectorBlue.getContours();
        Imgproc.drawContours(in, contoursB, -1, CONTOUR_COLOR_YELLOW_RGB, 1);

        mDetectorRed.process(mat2);
        Rect[] redBoundings = mDetectorRed.getBoundingRectangles();
        drawBoundingRect(in, redBoundings, CONTOUR_COLOR_RED_RGB);

        List<MatOfPoint> contoursR = mDetectorRed.getContours();
        Imgproc.drawContours(in, contoursR, -1, CONTOUR_COLOR_YELLOW_RGB, 1);

        beacon.processLights(blueBoundings, redBoundings, 0, 0);

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


        Imgproc.putText(in, "BeacAcq: " + beacon.isBeaconAcquired(), new Point(20, 200), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Imgproc.putText(in, "BeacSt: " + orientation, new Point(20, 220), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);
        Imgproc.putText(in, "BeacX: " + beacon.getX(), new Point(20, 240), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

        Imgproc.line(in, new Point(320,40), new Point(320,440), CONTOUR_COLOR_GREEN_RGB, 2);

        return in;
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

}
