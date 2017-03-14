package org.lapeerftcrobotics.sensor;

/**
 * Created by t6810sm on 11/23/2015.
 */
public class SensorListener implements DeviceOrientation.DeviceOrientationListener, DeviceAccel.DeviceAccelerometerListener {

    private static SensorListener instance = null;
    private float azimuth;
    private float pitch;
    private float roll;
    private float accelX;
    private float accelY;
    private float accelZ;

    public static synchronized SensorListener getInstance() {
        if (instance == null) {
            instance = new SensorListener();
        }
        return instance;
    }

    @Override
    public void onOrientationChanged(float azimuth, float pitch, float roll) {
        this.azimuth = azimuth;
        this.pitch = pitch;
        this.roll = roll;
    }

    public void onAccelChanged(float accelX, float accelY, float accelZ) {
        this.accelX = accelX;
        this.accelY = accelY;
        this.accelZ = accelZ;
    }

    public float getAzimuth() {
        return this.azimuth;
    }

    public float getPitch() {
        return this.pitch;
    }

    public float getRoll() {
        return this.roll;
    }

    public float getAccelX() {
        return this.accelX;
    }

    public float getAccelY() {
        return this.accelY;
    }

    public float getAccelZ() {
        return this.accelZ;
    }

}
