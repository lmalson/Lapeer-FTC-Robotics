package org.lapeerftcrobotics.vision;

import android.app.Activity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;

import com.qualcomm.ftccommon.DbgLog;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

/**
 * Created by T6810SM on 10/5/2016.
 */
public class OpenCVCameraActivityHelper {

    private AppUtil appUtil = AppUtil.getInstance();
    private CustomizedCameraView customizedCameraView;

    public OpenCVCameraActivityHelper() {

    }

    public void init(){
        appUtil.getActivity().runOnUiThread (new Thread(new Runnable() {
            public void run() {
                try {
                    Activity a = appUtil.getActivity();

                    customizedCameraView = new CustomizedCameraView(a,0);
                    customizedCameraView.setMinimumHeight(480);
                    customizedCameraView.setMinimumWidth(480);
                    ((ViewGroup)appUtil.getActivity().findViewById(R.id.cameraMonitorViewId)).addView(customizedCameraView);
                    customizedCameraView.setVisibility(View.VISIBLE);
                    customizedCameraView.setCvCameraViewListener(OpenCVCameraViewListener.getInstance());
                    OpenCVCameraViewListener.getInstance().setContext(appUtil.getActivity());

                    BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(a) {
                        @Override
                        public void onManagerConnected(int status) {
                            switch (status) {
                                case LoaderCallbackInterface.SUCCESS:
                                {
                                    DbgLog.msg("OpenCV loaded successfully");
                                    appUtil.showToast("OpenCV loaded successfully", Toast.LENGTH_SHORT);
//                    mOpenCvCameraView.setContext(context);
                                    customizedCameraView.enableView();
                                } break;
                                default:
                                {
                                    super.onManagerConnected(status);
                                } break;
                            }
                        }
                    };

                    OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, a, mLoaderCallback);

                } catch(Exception ex) {
                    System.out.print("Caught Exception: "+ex);
                    appUtil.showToast("OpenCV failed: "+ex, Toast.LENGTH_SHORT);
                }

            }
        }));
    }

    public void stop() {
        if (customizedCameraView != null) {
            customizedCameraView.disableView();
            appUtil.getActivity().runOnUiThread (new Thread(new Runnable() {
                public void run() {
                    try {
                        ((ViewGroup)appUtil.getActivity().findViewById(R.id.cameraMonitorViewId)).removeAllViews();
                    } catch(Exception ex) {
                        System.out.print("Caught Exception: "+ex);
                        appUtil.showToast("Failed Closing OpenCV: "+ex, Toast.LENGTH_SHORT);
                    }

                }
            }));
        }

    }


}
