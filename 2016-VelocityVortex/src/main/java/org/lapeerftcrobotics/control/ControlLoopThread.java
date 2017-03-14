package org.lapeerftcrobotics.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachineInterface;

/**
 * Created by t6810sm on 11/5/2015.
 */
public class ControlLoopThread extends Thread {
    private static long FAST_LOOP_INTERVAL_MS = 10;
    private long lastFastLoopTime = 0;
    private static long CONTROL_LOOP_INTERVAL_MS = 30;
    private boolean running = false;
    private long lastControlLoopTime = 0;
    private long cnt = 0;

    private AutonStateMachineInterface autonStateMachine = null;
    private RobotController robotController = null;

    public ControlLoopThread() {

    }

    public void setAutonStateMachineInterface(AutonStateMachineInterface a) {
        this.autonStateMachine = a;
    }

    public void setRobotController(RobotController rc) {
        this.robotController = rc;
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
                    autonStateMachine.process();
                    robotController.process();
                } catch (Exception ex) {
                }
            }
            try {
                 sleep(1);
            } catch (Exception e) {}
        }
    }

}
