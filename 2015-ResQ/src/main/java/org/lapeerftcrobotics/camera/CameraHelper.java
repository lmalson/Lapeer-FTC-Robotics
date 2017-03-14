package org.lapeerftcrobotics.camera;

import android.app.Activity;
import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.Size;
import android.widget.FrameLayout;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.log.FileLogger;

import java.util.List;

/**
 * Created by t6810sm on 10/15/2015.
 *
 * In Activity
 *
 * CameraHelper h = new CameraHelper(this);
 * h.openFrontFacingCamera();
 * h.initPreview((FrameLayout)findViewById(R.id.previewLayout));
 *
 */
public class CameraHelper {

    private static CameraHelper instance = null;

    private Camera camera;
    private Context activityContext;
    private ImageHandler imageHandler;
    private FileLogger fileLogger;

    /**
     * Call from Activity
     */
    public CameraHelper(Context c) {
        this.instance = this;
        this.activityContext = c;
        this.imageHandler = new ImageHandler(this);
    }

    public static CameraHelper getInstance() {
        return instance;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public FileLogger getFileLogger() {
        return this.fileLogger;
    }

    public ImageHandler getImageHandler() {
        return this.imageHandler;
    }

    /**
     * Call from Activity
     */
    public void openFrontFacingCamera() {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {

        }
        this.camera = cam;
    }

    /**
     * Call from Activity
     *
     */

    public void initPreview(FrameLayout previewLayout) {
           CameraPreview p = new CameraPreview(activityContext, camera, imageHandler, this);
           previewLayout.addView(p);
    }
}
