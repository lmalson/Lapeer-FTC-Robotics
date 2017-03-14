package org.lapeerftcrobotics.sensor;


import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.view.WindowManager;

/**
 * Created by t6810sm on 5/19/2015.
 *
 * values[0]: azimuth, rotation around the Z axis.
 * values[1]: pitch, rotation around the X axis.
 * values[2]: roll, rotation around the Y axis.
 */
public class DeviceAccel implements SensorEventListener {

    private static final int SENSOR_DELAY_MICROS = 30 * 1000; // 30ms
    private static final int CALIBRATION_STATE = 0;
    private static final int READ_STATE = 1;
    private static final int CALIBRATION_LOOPS = 100;
    private final SensorManager mSensorManager;
    private final Sensor mAccelerometerSensor;
    private DeviceAccelerometerListener mListener;
    private int mLastAccuracy;

    private int state = CALIBRATION_STATE;
    private int calibrationCnt = CALIBRATION_LOOPS;
    private double calibrationAccumX = 0;
    private double calibrationAccumY = 0;
    private double calibrationAccumZ = 0;
    private float offsetX = 0;
    private float offsetY = 0;
    private float offsetZ = 0;

    public interface DeviceAccelerometerListener {
        void onAccelChanged(float accelX, float accelY, float accelZ);
    }

    public DeviceAccel(SensorManager sensorManager) {
        mSensorManager = sensorManager;

        // Can be null if the sensor hardware is not available
        mAccelerometerSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    public void registerListener(DeviceAccelerometerListener listener) {
        if (mListener == listener) {
            return;
        }
        mListener = listener;
        if (mAccelerometerSensor == null) {
            Log.w("DeviceAccel","Accelerometer sensor not available; will not provide accelerometer data.");
            return;
        }
        mSensorManager.registerListener(this, mAccelerometerSensor, SENSOR_DELAY_MICROS);
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
        if (event.sensor == mAccelerometerSensor) {
            updateAcceleration(event.values);
        }
    }

    private void updateAcceleration(float[] accelVector) {
        float xRaw = accelVector[0];
        float yRaw = accelVector[1];
        float zRaw = accelVector[2];

        switch(state) {
            case CALIBRATION_STATE:
                if (calibrationCnt == 0) {
                    offsetX = (float)(calibrationAccumX / CALIBRATION_LOOPS);
                    offsetY = (float)(calibrationAccumY / CALIBRATION_LOOPS);
                    offsetZ = (float)(calibrationAccumZ / CALIBRATION_LOOPS);
                    this.state = READ_STATE;
                } else
                {
                    calibrationAccumX += xRaw;
                    calibrationAccumY += yRaw;
                    calibrationAccumZ += zRaw;
                    calibrationCnt--;
                }
                break;

            case READ_STATE:
                float x = xRaw - offsetX;
                float y = yRaw - offsetY;
                float z = zRaw - offsetZ;
                mListener.onAccelChanged(x, y, z);
                break;
        }

    }

}
