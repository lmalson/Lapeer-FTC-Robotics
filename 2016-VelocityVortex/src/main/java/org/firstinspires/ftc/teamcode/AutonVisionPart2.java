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
import com.qualcomm.robotcore.util.ElapsedTime;

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

//@Autonomous(name="AutonVision2", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class AutonVisionPart2 extends OpMode implements OnCameraFrameCallbackInterface
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCVCameraActivityHelper openCVCameraActivityHelper = null;

    private static Scalar RED_LOWER_BOUNDS_HSV = new Scalar(0,100,150);
    private static Scalar RED_UPPER_BOUNDS_HSV = new Scalar(30,255,255);
    private static Scalar RED_LOWER_BOUNDS2_HSV = new Scalar(225,100,150);
    private static Scalar RED_UPPER_BOUNDS2_HSV = new Scalar(255,255,255);

    private final static Scalar CONTOUR_COLOR_RED_RGB = new Scalar(255,0,0,255);
    private final static Scalar CONTOUR_COLOR_GREEN_RGB = new Scalar(0,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLUE_RGB = new Scalar(0,0,255,255);
    private final static Scalar CONTOUR_COLOR_WHITE_RGB = new Scalar(255,255,255,255);
    private final static Scalar CONTOUR_COLOR_YELLOW_RGB = new Scalar(255,255,0,255);
    private final static Scalar CONTOUR_COLOR_BLACK_RGB = new Scalar(0,0,0,255);

    private Mat mat1 = null;
    private Mat mat2 = null;
    private Mat mat3 = null;
    private Mat mat4 = null;
    private Mat mat5 = null;
    private Mat mat6 = null;
    private Mat mat7 = null;
    private Mat mHierarchy = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        openCVCameraActivityHelper = new OpenCVCameraActivityHelper();
        openCVCameraActivityHelper.init();

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
        mat1 = new Mat(480,320, CvType.CV_8UC4);
        mat2 = new Mat(480,320, CvType.CV_8UC4);
        mat3 = new Mat(480,320, CvType.CV_8UC4);
        mat4 = new Mat(480,320, CvType.CV_8UC4);
        mat5 = new Mat(480,320, CvType.CV_8UC4);
        mat6 = new Mat(480,320, CvType.CV_8UC4);
        mat7 = new Mat(480,320, CvType.CV_8UC4);
        mHierarchy = new Mat();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
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

        Core.inRange(mat2, RED_LOWER_BOUNDS_HSV, RED_UPPER_BOUNDS_HSV, mat3);
        Core.inRange(mat2, RED_LOWER_BOUNDS2_HSV, RED_UPPER_BOUNDS2_HSV, mat4);

        Core.bitwise_or(mat3, mat4, mat5);

        Imgproc.dilate(mat5, mat6, new Mat());
        Imgproc.dilate(mat6, mat7, new Mat()); //fill in holes

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        // find countours(shapes)
        Imgproc.findContours(mat7, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //draw - on mat1 image
        Imgproc.drawContours(mat1, contours, -1, CONTOUR_COLOR_YELLOW_RGB, 3);

        return mat1;
    }

}
