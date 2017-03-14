package org.lapeerftcrobotics.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachine;
import org.lapeerftcrobotics.auton.Beacon;
import org.lapeerftcrobotics.log.FileLogger;

/**
 * Created by t6810sm on 11/5/2015.
 */
public class ControlLoopThread extends Thread {
    private static long FAST_LOOP_INTERVAL_MS = 10;
    private long lastFastLoopTime = 0;
    private static long CONTROL_LOOP_INTERVAL_MS = 30;
    private boolean running = false;
    private FileLogger fileLogger;
    private long lastControlLoopTime = 0;
    private long cnt = 0;

    private BeaconTargetController beaconTargetController = null;
    private AutonStateMachine autonStateMachine = null;
    private RobotController robotController = null;
    private RampTargetController rampTargetController = null;

    public ControlLoopThread() {

    }

    public void setBeaconTargetController(BeaconTargetController btc) {
        this.beaconTargetController = btc;
    }

    public void setRampTargetController(RampTargetController rtc) {
        this.rampTargetController = rtc;
    }

    public void setAutonStateMachine(AutonStateMachine a) {
        this.autonStateMachine = a;
    }

    public void setRobotController(RobotController rc) {
        this.robotController = rc;
    }

    public void setFileLogger(FileLogger fileLogger) {
        this.fileLogger = fileLogger;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public void run() {
        super.run();
        long startTime;

        while(running) {
            startTime = System.currentTimeMillis();

            if ((startTime - lastFastLoopTime) >= FAST_LOOP_INTERVAL_MS) {
                lastFastLoopTime = startTime;
                // Read Gyro every 10ms
                robotController.processFast();
            }
            if ((startTime - lastControlLoopTime) >= CONTROL_LOOP_INTERVAL_MS)
            {
                long controlLoopPeriod = startTime - lastControlLoopTime;
                lastControlLoopTime = startTime;
                cnt++;

                try {
                    beaconTargetController.process();
                    rampTargetController.process();
                    autonStateMachine.process();
                    robotController.process();
                } catch (Exception ex) {
                    if (fileLogger != null)
                        fileLogger.writeEvent("ControlLoopThread.run()","XXXXX Exception: "+ex.getCause());
                }

//                if (fileLogger != null)
//                    fileLogger.writeEvent("ControlLoopThread.run()","cnt: "+cnt+" loopPeriod: "+controlLoopPeriod);
            }
            try {
                 sleep(1);
            } catch (Exception e) {}
        }
    }
}
