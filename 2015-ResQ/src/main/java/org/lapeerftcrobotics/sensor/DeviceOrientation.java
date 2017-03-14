package org.lapeerftcrobotics.sensor;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.view.Surface;
import android.view.WindowManager;

/**
 * Created by t6810sm on 5/19/2015.
 *
 * values[0]: azimuth, rotation around the Z axis.
 * values[1]: pitch, rotation around the X axis.
 * values[2]: roll, rotation around the Y axis.
 */
public class DeviceOrientation implements SensorEventListener {

    private static final float RAD_TO_DEG = (float)(180.0 / (Math.PI));
    private static final int SENSOR_DELAY_MICROS = 30 * 1000; // 30ms
    private static final int CALIBRATION_STATE = 0;
    private static final int READ_STATE = 1;
    private static final int CALIBRATION_LOOPS = 100;
    private final SensorManager mSensorManager;
    private final Sensor mRotationSensor;
    private final WindowManager mWindowManager;
    private DeviceOrientationListener mListener;
    private int mLastAccuracy;

    private int state = CALIBRATION_STATE;
    private int calibrationCnt = CALIBRATION_LOOPS;
    private double calibrationAccumAzimuth = 0;
    private double calibrationAccumPitch = 0;
    private double calibrationAccumRoll = 0;
    private float offsetAzimuth = 0;
    private float offsetPitch = 0;
    private float offsetRoll = 0;

    public interface DeviceOrientationListener {
        void onOrientationChanged(float azimuth, float pitch, float roll);
    }

    public DeviceOrientation(SensorManager sensorManager, WindowManager windowManager) {
        mSensorManager = sensorManager;
        mWindowManager = windowManager;

        // Can be null if the sensor hardware is not available
        mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    public void registerListener(DeviceOrientationListener listener) {
        if (mListener == listener) {
            return;
        }
        mListener = listener;
        if (mRotationSensor == null) {
            Log.w("DeviceAccel","Rotation vector sensor not available; will not provide orientation data.");
            return;
        }
        mSensorManager.registerListener(this, mRotationSensor, SENSOR_DELAY_MICROS);
    }

    public void removeListener() {
        mSensorManager.unregisterListener(this);
        mListener = null;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        if (mLastAccuracy != accuracy) {
            mLastAccuracy = accuracy;
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (mListener == null) {
            return;
        }
        if (mLastAccuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
            return;
        }
        if (event.sensor == mRotationSensor) {
            updateOrientation(event.values);
        }
    }

    private void updateOrientation(float[] rotationVector) {
        float[] rotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(rotationMatrix, rotationVector);

        // By default, remap the axes as if the front of the
        // device screen was the instrument panel.
        int worldAxisForDeviceAxisX = SensorManager.AXIS_MINUS_Z;
        int worldAxisForDeviceAxisY = SensorManager.AXIS_X;

        float[] adjustedRotationMatrix = new float[9];
        SensorManager.remapCoordinateSystem(rotationMatrix, worldAxisForDeviceAxisX,
                worldAxisForDeviceAxisY, adjustedRotationMatrix);

        // Transform rotation matrix into azimuth/pitch/roll
        float[] orientation = new float[3];
        SensorManager.getOrientation(adjustedRotationMatrix, orientation);

        // Convert radians to degrees
        float azimuthRaw = orientation[0] * RAD_TO_DEG;
        float pitchRaw = orientation[1] * -RAD_TO_DEG;
        float rollRaw = orientation[2] * RAD_TO_DEG;

        switch(state) {
            case CALIBRATION_STATE:
                if (calibrationCnt == 0) {
                    offsetAzimuth = (float)(calibrationAccumAzimuth / CALIBRATION_LOOPS);
                    offsetPitch = (float)(calibrationAccumPitch / CALIBRATION_LOOPS);
                    offsetRoll = (float)(calibrationAccumRoll / CALIBRATION_LOOPS);
                    this.state = READ_STATE;
                } else
                {
                    calibrationAccumAzimuth += azimuthRaw;
                    calibrationAccumPitch += pitchRaw;
                    calibrationAccumRoll += rollRaw;
                    calibrationCnt--;
                }
                break;

            case READ_STATE:
                float azimuth = azimuthRaw - offsetAzimuth;
                if (azimuth > 180.0)
                    azimuth -= 360.0;
                else if (azimuth <-180.0)
                    azimuth += 360.0;
                float pitch = pitchRaw - offsetPitch;
                if (pitch > 180.0)
                    pitch -= 360.0;
                else if (pitch <-180.0)
                    pitch += 360.0;
                float roll = rollRaw - offsetRoll;
                if (roll > 180.0)
                    roll -= 360.0;
                else if (roll <-180.0)
                    roll += 360.0;
                mListener.onOrientationChanged(azimuth, pitch, roll);
                break;
        }

    }

}
