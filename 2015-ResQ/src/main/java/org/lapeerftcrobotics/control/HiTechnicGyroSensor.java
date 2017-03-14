package org.lapeerftcrobotics.control;

import com.qualcomm.robotcore.hardware.GyroSensor;

import org.lapeerftcrobotics.auton.AutonStateMachine;
import org.lapeerftcrobotics.log.FileLogger;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by t6810sm on 11/5/2015.
 */
public class HiTechnicGyroSensor extends Thread {
    private final static long FAST_LOOP_INTERVAL_MS = 10;
    private final static int SAMPLE_COUNT = 200;
    private long lastFastLoopTime = 0;
    private boolean running = false;
    private long cnt = 0;
    private int gyroState = 0;
    private GyroSensor gyro;
    private int sampleCnt = SAMPLE_COUNT;
    private int calMinValue = 1024;
    private int calMaxValue = 0;
    private long zeroOffset = 0;
    private int deadBand = 0;
    private int headingRaw = 0;
    private double heading = 0;
    private FileLogger fileLogger = null;
    private boolean debugLog = false;

    public HiTechnicGyroSensor() {

    }

    public void setDebugLogging(boolean val) {
        this.debugLog = val;
    }

    public int getHeadingRaw() {
        return this.headingRaw;
    }

    public int getGyroState() {
        return this.gyroState;
    }

    public long getOffset() {
        return this.zeroOffset;
    }

    public int getDeadBand() {
        return this.deadBand;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public void setGyro(GyroSensor g) {
        this.gyro = g;
    }

    public void reset() {
        this.heading = 0.0;
    }

    public double getHeading() {
        return this.heading;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public void run() {
        super.run();
        long startTime;
        cnt++;

        while(running) {
            try {
                startTime = System.currentTimeMillis();

                if ((startTime - lastFastLoopTime) >= FAST_LOOP_INTERVAL_MS) {
                    lastFastLoopTime = startTime;
                    // Read Gyro every 10ms

                    switch (gyroState) {
                        case 0: { // calibration

                            if (sampleCnt > 0) {
                                int rate = (int) gyro.getRotation();
                                this.zeroOffset += rate;
                                if (rate < calMinValue) {
                                    calMinValue = rate;
                                } else if (rate > calMaxValue) {
                                    calMaxValue = rate;
                                }
                                sampleCnt--;
                                if (debugLog) fileLogger.writeEvent("gyro.run()", " state: " + gyroState + " sampleCnt: " + sampleCnt + " zeroOffset: " + zeroOffset + " minVal: " + calMinValue + " maxVal: " + calMaxValue);
                            } else {
                                this.zeroOffset /= SAMPLE_COUNT;
                                this.deadBand = calMaxValue - calMinValue;
                                gyroState = 1; // completed calibration
                                if (debugLog) fileLogger.writeEvent("gyro.run()", " state: " + gyroState + " sampleCnt: " + sampleCnt + " zeroOffset: " + zeroOffset + " minVal: " + calMinValue + " maxVal: " + calMaxValue + " deadband: " + deadBand);
                            }
                            break;
                        }
                        case 1: { // read gyro
                            int rate = (int) gyro.getRotation();
                            if (debugLog) fileLogger.writeEvent("gyro.run()", " state: " + gyroState + " rate: " + rate);
                            rate -= this.zeroOffset;
                            if (rate > -this.deadBand && rate < this.deadBand) {
                                rate = 0;
                            }
                            if (debugLog) fileLogger.writeEvent("gyro.run()", " state: " + gyroState + " rate2: " + rate);
                            this.headingRaw += rate;
                            double h = this.heading + rate * (FAST_LOOP_INTERVAL_MS) / 1000.0;
                            if (h > 360.0)
                                h -= 360.0;
                            else if (h < -360.0)
                                h += 360.0;
                            this.heading = h;
                            if (debugLog) fileLogger.writeEvent("gyro.run()", " heading: " + heading);
                            break;
                        }
                    }
                }
            } catch (Exception ex) {
                if (fileLogger != null) {
                    fileLogger.writeEvent("gyro.run()", "XXXXX Exception: " +ex.toString());
                    StringWriter sw = new StringWriter();
                    ex.printStackTrace(new PrintWriter(sw));
                    String exceptionAsString = sw.toString();
                    fileLogger.writeEvent("gyro.run()", "XXXXX Trace: " +exceptionAsString);
            }

            }

            try {
                 sleep(1);
            } catch (Exception e) {}
        }
    }
}
