package org.lapeerftcrobotics.vision;

import android.app.Activity;
import android.content.Context;
import android.widget.Toast;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class OpenCVCameraViewListener implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static OpenCVCameraViewListener instance;
    private boolean initialized = false;
    private Mat mRgba;
    private OnCameraFrameCallbackInterface onCameraFrameCallbackListener;
    private Context context;
    private int frameCnt = 0;

    @Override
    public void onCameraViewStarted(int width, int height) {

        mRgba = new Mat(height, width, CvType.CV_8UC4);

        initialized = true;
    }

    public void registerOnFrameCallbackListener(OnCameraFrameCallbackInterface c) {
        this.onCameraFrameCallbackListener = c;
    }

    public void setContext(Context c) {
        this.context = c;
    }

    public static synchronized OpenCVCameraViewListener getInstance() {
        if (instance == null) {
            instance = new OpenCVCameraViewListener();
        }
        return instance;
    }

    @Override
    public void onCameraViewStopped() {
    }

    public void showToast(final Toast toast) {
        ((Activity)context).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                toast.show();
            }
        });
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        try {

            mRgba=inputFrame.rgba();

            if (!initialized)
                return mRgba;

            frameCnt++;

            if ((frameCnt % 50 == 0) && (context != null)) {
//                Toast toast = Toast.makeText(context, "onCameraFrame "+frameCnt, Toast.LENGTH_LONG);
//                toast.setGravity(Gravity.CENTER, 0, 0);
//                showToast(toast);
            }

            if (this.onCameraFrameCallbackListener != null) {
                return this.onCameraFrameCallbackListener.process(mRgba);
            }

        } catch (Exception ex) {
            System.out.println("Exception ex: "+ex);
        }

        return mRgba;
    }
}

