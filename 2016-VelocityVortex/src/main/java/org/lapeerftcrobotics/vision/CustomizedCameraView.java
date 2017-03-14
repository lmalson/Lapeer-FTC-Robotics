package org.lapeerftcrobotics.vision;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.os.Build;
import android.util.AttributeSet;
import android.util.Log;

import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * This class is an implementation of the Bridge View between OpenCV and Java Camera. This class relays on the
 * functionality available in base class and only implements required functions: connectCamera - opens Java camera and
 * sets the PreviewCallback to be delivered. disconnectCamera - closes the camera and stops preview. When frame is
 * delivered via callback from Camera - it processed via OpenCV to be converted to RGBA32 and then passed to the
 * external callback for modifications if required.
 */
@SuppressLint("NewApi")
public class CustomizedCameraView extends CameraBridgeViewBase implements PreviewCallback {

    private static final int MAGIC_TEXTURE_ID = 10;
    private static final String TAG = "CustomizedCameraView";

    private byte mBuffer[];
    private Mat[] mFrameChain;
    private int mChainIdx = 0;
    private Thread mThread;
    private boolean mStopThread;

    protected Camera mCamera;
    protected JavaCameraFrame[] mCameraFrame;
    private SurfaceTexture mSurfaceTexture;

    public static class JavaCameraSizeAccessor implements ListItemAccessor {

        public int getWidth(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.width;
        }

        public int getHeight(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.height;
        }
    }

    public CustomizedCameraView(Context context, int cameraId) {
        super(context, cameraId);
    }

    public CustomizedCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    protected boolean initializeCamera(int width, int height) {
        Log.d(TAG, "Initialize java camera");
        boolean result = true;
        synchronized (this) {
            mCamera = null;

                    int localCameraIndex = mCameraIndex;
                        Log.d(TAG, "Trying to open camera with new open(" + Integer.valueOf(localCameraIndex) + ")");
                        try {

                            // 0 = BACK
                            // 1 = FRONT
                            mCamera = Camera.open(1); //localCameraIndex);
                        } catch (RuntimeException e) {
                            Log.e(TAG, "Camera #" + localCameraIndex + "failed to open: " + e.getLocalizedMessage());
                        }
//                    }
//                }
//            }

            if (mCamera == null)
                return false;


            /* Now set camera parameters */
            try {
                Camera.Parameters params = mCamera.getParameters();

//                Log.d(TAG, "getSupportedPreviewSizes()");
                List<Camera.Size> sizes = params.getSupportedPreviewSizes();

                for(int i=0; i<sizes.size(); i++) {
                    System.out.println(" Size["+i+"] w: "+sizes.get(i).width+" h: "+sizes.get(i).height);
                }

                if (sizes != null) {
                    /* Select the size that fits surface considering maximum size allowed */
//                    Size frameSize = calculateCameraFrameSize(sizes, new JavaCameraSizeAccessor(), width, height);

                    params.setPreviewFormat(ImageFormat.NV21);
//                    Log.d(TAG,
//                            "Set preview size to " + Integer.valueOf((int) frameSize.width) + "x"
//                                    + Integer.valueOf((int) frameSize.height));
//                    params.setPreviewSize((int) frameSize.width, (int) frameSize.height);

//                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.ICE_CREAM_SANDWICH)
//                        params.setRecordingHint(true);

// 4                   List<String> FocusModes = params.getSupportedFocusModes();
//                    if (FocusModes != null && FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO)) {
//                        params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
//                    }

                    // 320x240
                    // 352x288
                    // 384x288
                    // 480x320
                    // 576x432
                    // 640x480
                    // 720x480


                    // Samsung s5
                    // 640 x 480
                    // 352 x 288

                    // http://stackoverflow.com/questions/19235477/google-glass-preview-image-scrambled-with-new-xe10-release/

                    if (ImageProcessingManager.PHONE_TYPE == ImageProcessingManager.ZTE) {
                        params.setPreviewFpsRange(20000, 20000);  // 10000,10000
                        params.setPreviewSize(480, 320);
                        params.setExposureCompensation(-12);
                        params.setAutoExposureLock(true);
                    }
                    else {
                        params.setPreviewFpsRange(20000, 20000);  // 10000,10000
                        params.setPreviewSize(640,480);
                        params.setExposureCompensation(-2);
                        params.setAutoExposureLock(true);
                    }

                    mCamera.setParameters(params);
                    params = mCamera.getParameters();

                    mFrameWidth = params.getPreviewSize().width;
                    mFrameHeight = params.getPreviewSize().height;

//                    if ((getLayoutParams().width == LayoutParams.MATCH_PARENT)
//                            && (getLayoutParams().height == LayoutParams.MATCH_PARENT))
//                        mScale = Math.min(((float) height) / mFrameHeight, ((float) width) / mFrameWidth);
//                    else
// APPLY NO SCALING

                        mScale = 0;

                    if (mFpsMeter != null) {
                        mFpsMeter.setResolution(mFrameWidth, mFrameHeight);
                    }

                    int size = mFrameWidth * mFrameHeight;
                    size = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                    mBuffer = new byte[size];

                    mCamera.addCallbackBuffer(mBuffer);
                    mCamera.setPreviewCallbackWithBuffer(this);

                    mFrameChain = new Mat[2];
                    mFrameChain[0] = new Mat(mFrameHeight + (mFrameHeight / 2), mFrameWidth, CvType.CV_8UC1);
                    mFrameChain[1] = new Mat(mFrameHeight + (mFrameHeight / 2), mFrameWidth, CvType.CV_8UC1);

                    AllocateCache();

                    mCameraFrame = new JavaCameraFrame[2];
                    mCameraFrame[0] = new JavaCameraFrame(mFrameChain[0], mFrameWidth, mFrameHeight);
                    mCameraFrame[1] = new JavaCameraFrame(mFrameChain[1], mFrameWidth, mFrameHeight);

                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
                        mSurfaceTexture = new SurfaceTexture(MAGIC_TEXTURE_ID);
                        mCamera.setPreviewTexture(mSurfaceTexture);
                    } else
                        mCamera.setPreviewDisplay(null);

                    /* Finally we are ready to start the preview */
                    Log.d(TAG, "startPreview");

                    params = mCamera.getParameters();
                    params.set("orientation", "portrait");
                    mCamera.setParameters(params);

                    mCamera.startPreview();
                } else
                    result = false;
            } catch (Exception e) {
                result = false;
                e.printStackTrace();
            }
        }

        return result;
    }

    protected void releaseCamera() {
        synchronized (this) {
            if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);

                mCamera.release();
            }
            mCamera = null;
            if (mFrameChain != null) {
                mFrameChain[0].release();
                mFrameChain[1].release();
            }
            if (mCameraFrame != null) {
                mCameraFrame[0].release();
                mCameraFrame[1].release();
            }
        }
    }

    @Override
    protected boolean connectCamera(int width, int height) {

        /*
         * 1. We need to instantiate camera 2. We need to start thread which will be getting frames
         */
        /* First step - initialize camera connection */
        Log.d(TAG, "Connecting to camera");
        if (!initializeCamera(width, height))
            return false;

        /* now we can start update thread */
        Log.d(TAG, "Starting processing thread");
        mStopThread = false;
        mThread = new Thread(new CameraWorker());
        mThread.start();

        return true;
    }

    protected void disconnectCamera() {
        /*
         * 1. We need to stop thread which updating the frames 2. Stop camera and release it
         */
        Log.d(TAG, "Disconnecting from camera");
        try {
            mStopThread = true;
            Log.d(TAG, "Notify thread");
            synchronized (this) {
                ((Object)this).notify();
            }
            Log.d(TAG, "Wating for thread");
            if (mThread != null)
                mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mThread = null;
        }

        /* Now release camera */
        releaseCamera();
    }

    public void onPreviewFrame(byte[] frame, Camera arg1) {
//        Log.d(TAG, "Preview Frame received. Frame size: " + frame.length);

        synchronized (this) {
            mFrameChain[1 - mChainIdx].put(0, 0, frame);
            ((Object)this).notify();
        }
        if (mCamera != null)
            mCamera.addCallbackBuffer(mBuffer);
    }

    private class JavaCameraFrame implements CvCameraViewFrame {
        public Mat gray() {
            return mYuvFrameData.submat(0, mHeight, 0, mWidth);
        }

        public Mat rgba() {
            Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2BGR_NV12, 4);
            return mRgba;
        }

        public JavaCameraFrame(Mat Yuv420sp, int width, int height) {
            super();
            mWidth = width;
            mHeight = height;
            mYuvFrameData = Yuv420sp;
            mRgba = new Mat();
        }

        public void release() {
            mRgba.release();
        }

        private Mat mYuvFrameData;
        private Mat mRgba;
        private int mWidth;
        private int mHeight;
    };

    private class CameraWorker implements Runnable {

        public void run() {
            do {
                synchronized (CustomizedCameraView.this) {
                    try {
                        ((Object)CustomizedCameraView.this).wait();
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }

                if (!mStopThread) {
                    if (!mFrameChain[mChainIdx].empty())
                        deliverAndDrawFrame(mCameraFrame[mChainIdx]);
                    mChainIdx = 1 - mChainIdx;
                }
            } while (!mStopThread);
            Log.d(TAG, "Finish processing thread");
        }
    }

    public float getScale() {
        return mScale;
    }

}
