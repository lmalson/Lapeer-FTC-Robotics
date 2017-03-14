package org.lapeerftcrobotics.camera;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.log.FileLogger;

import java.io.ByteArrayOutputStream;
import java.io.FileOutputStream;

/**
 * Created by t6810sm on 10/15/2015.
 */
public class ImageHandler implements Camera.PreviewCallback {
    private Bitmap image;
    private int width = 640;
    private int height = 480;
    private YuvImage yuvImage = null;
    private String data;
    private boolean imageProcessed = false;
    private CameraHelper parent;
    private int imgCnt = 0;

    public ImageHandler(CameraHelper parent) {
        this.parent = parent;
    }

    public void onPreviewFrame(byte[] data, Camera camera)
    {
//        Camera.Parameters parameters = camera.getParameters();
//        width = parameters.getPreviewSize().width;
//        height = parameters.getPreviewSize().height;

        long startTime = System.currentTimeMillis();

        Bitmap img = convertImage(data,640,480);

        if (parent.getFileLogger() != null) {
            parent.getFileLogger().saveBitmap(img,"-img"+imgCnt+".txt");
            imgCnt++;
        }
        else
        {
           imgCnt = 0;
        }

        int redValue = 0;
        int blueValue = 0;
        int greenValue = 0;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                int pixel = img.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
                greenValue += green(pixel);
            }
        }

        long dur = System.currentTimeMillis() - startTime;
        if (parent.getFileLogger() != null)
            parent.getFileLogger().writeEvent("imageHandler.onPreviewFrame()", "width: " + width + " height: " + height + " dur(ms): " + dur);

    }

    private Bitmap convertImage(byte[] data, int width, int height) {
        YuvImage img = new YuvImage(data, ImageFormat.NV21, width, height, null);  // NV21 is default (17)
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        img.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        return BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }





    public void processImage() {

        if (parent.getFileLogger() != null)
            parent.getFileLogger().writeEvent("imageHandler.processImage()", " ");
    }

    public void processImageX() {
        if (!imageProcessed) {
            int redValue = 0;
            int blueValue = 0;
            int greenValue = 0;
//            convertImage();
            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    int pixel = image.getPixel(x, y);
                    redValue += red(pixel);
                    blueValue += blue(pixel);
                    greenValue += green(pixel);
                }
            }
            data = "Red: "+redValue+", Blue: "+blueValue+", Green: "+greenValue;
        }
    }






    public String getData() {
        return this.data;
    }

    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }

    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    private int blue(int pixel) {
        return pixel & 0xff;
    }

}
