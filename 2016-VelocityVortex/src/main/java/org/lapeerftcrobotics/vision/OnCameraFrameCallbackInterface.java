package org.lapeerftcrobotics.vision;

import org.opencv.core.Mat;

public interface OnCameraFrameCallbackInterface {
    public Mat process(Mat mRgba);
}
