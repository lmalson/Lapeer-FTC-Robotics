package org.lapeerftcrobotics.camera;

import android.content.Context;
import android.graphics.ImageFormat;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.hardware.Camera;

import java.io.IOException;
import java.util.List;

/**
 *
 */
public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback {
    private SurfaceHolder mHolder;
    private Camera mCamera;
    private static String TAG = "DEBUG";
    private Camera.PreviewCallback previewCallback;
    private CameraHelper parent;

    public CameraPreview(Context context, Camera camera, Camera.PreviewCallback previewCallback, CameraHelper parent) {
        super(context);
        mCamera = camera;

        this.previewCallback = previewCallback;
        this.parent = parent;

        // Install a SurfaceHolder.Callback so we get notified when the
        // underlying surface is created and destroyed.
        mHolder = getHolder();
        mHolder.addCallback(this);
        // deprecated setting, but required on Android versions prior to 3.0
//        mHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, now tell the camera where to draw the preview.
        try {
            mCamera.setPreviewCallback(previewCallback);
            mCamera.setPreviewDisplay(holder);

            Camera.Parameters parameters = mCamera.getParameters();

            // set to width:640, height:480
            parameters.setPreviewSize(640,480);
            parameters.setPreviewFpsRange(8000,8000);
            parameters.setRotation(0);
            mCamera.setParameters(parameters);

            Camera.Size s= parameters.getPreviewSize();
            int wishedBufferSize=s.height * s.width * 3 / 2;
            mCamera.addCallbackBuffer(new byte[wishedBufferSize]);
            mCamera.addCallbackBuffer(new byte[wishedBufferSize]);

            mCamera.startPreview();
        } catch (IOException e) {
            Log.d(TAG, "Error setting camera preview: " + e.getMessage());
        }
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        // empty. Take care of releasing the Camera preview in your activity.
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
        // If your preview can change or rotate, take care of those events here.
        // Make sure to stop the preview before resizing or reformatting it.

        if (mHolder.getSurface() == null){
            // preview surface does not exist
            return;
        }

        // stop preview before making changes
        try {
            mCamera.stopPreview();
        } catch (Exception e){
            // ignore: tried to stop a non-existent preview
        }

        // set preview size and make any resize, rotate or
        // reformatting changes here

        // start preview with new settings
        try {
            mCamera.setPreviewCallback(previewCallback);
            mCamera.setPreviewDisplay(mHolder);
            mCamera.startPreview();

        } catch (Exception e){
            Log.d(TAG, "Error starting camera preview: " + e.getMessage());
        }
    }
}



//            List<Integer> frameRates=parameters.getSupportedPreviewFrameRates();
//            for(Integer i : frameRates) {
//                System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX rate: "+i.toString());
//            }
/**
 *
 *             List<Integer> formats=parameters.getSupportedPreviewFormats();
 for(Integer i : formats) {
 System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX format: "+i.toString());
 }

 List<Camera.Size> sizes = parameters.getSupportedPreviewSizes();
 String sizeInfo = "";
 int i=0;
 for (Camera.Size size : sizes) {
 sizeInfo += "\n["+i+"] width: "+size.width+", height: "+size.height;
 i++;
 }
 System.out.println("\n\ncameraPreview.surfaceCreated() "+sizeInfo+"\n\n\n");
 if (parent.getFileLogger() != null && parent.getRuntime() != null) {
 parent.getFileLogger().write(parent.getRuntime().toString()+","+System.currentTimeMillis() + "," + "cameraPreview.surfaceCreated()" + "," + sizeInfo);
 }
 */
