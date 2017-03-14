package org.lapeerftcrobotics.vision;

import org.opencv.core.Mat;

public class OnCameraFrameCallbackListenerImplBase implements OnCameraFrameCallbackInterface {
    public Mat process(Mat mRgba) {
        return mRgba;
    }
}
