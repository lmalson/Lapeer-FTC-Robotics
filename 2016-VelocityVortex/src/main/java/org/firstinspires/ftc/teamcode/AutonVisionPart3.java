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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 *
 */

//@Autonomous(name="AutonVision3", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class AutonVisionPart3 extends OpMode implements OnCameraFrameCallbackInterface
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCVCameraActivityHelper openCVCameraActivityHelper = null;

    private static Scalar RED_LOWER_BOUNDS_HSV = new Scalar(0,60,60);
    private static Scalar RED_UPPER_BOUNDS_HSV = new Scalar(45,255,255);
    private static Scalar RED_LOWER_BOUNDS2_HSV = new Scalar(210,60,60);
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

    String hsv = "";
    int imgCnt = 0;

    Scalar hsvColor = new Scalar(0,0,0,0);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        openCVCameraActivityHelper = new OpenCVCameraActivityHelper();
        openCVCameraActivityHelper.init();

//        OpenCVCameraViewListener.getInstance().registerOnFrameCallbackListener(this);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("HSV", " " + hsv);
        telemetry.addData("Img", " " + imgCnt);
        telemetry.update();

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

        imgCnt++;

        try {
            Mat m2 = mat2.submat(new Rect(235,155,5,5));
            hsvColor = Core.sumElems(m2);

//            Imgproc.putText(in,"hsvColor l: "+hsvColor.val.length,new Point(10, 220), 3, 0.5, CONTOUR_COLOR_WHITE_RGB, 1);

            for (int i = 0; i < hsvColor.val.length; i++)
                hsvColor.val[i] /= 25; // pointCount;

            hsv = "H: "+hsvColor.val[0]+" S: "+hsvColor.val[1]+ " V: " + hsvColor.val[2];
        } catch (Exception ex) {
            hsv = "Ex: "+ex;
        }

        Imgproc.rectangle(in, new Point(235, 155), new Point(240, 160), CONTOUR_COLOR_YELLOW_RGB, 1);

        Imgproc.putText(in, hsv, new Point(20, 10), 3, 1.5, CONTOUR_COLOR_WHITE_RGB, 1);

        return in;
    }

}
