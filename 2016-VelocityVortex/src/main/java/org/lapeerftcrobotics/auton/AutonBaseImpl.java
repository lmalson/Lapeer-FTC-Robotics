package org.lapeerftcrobotics.auton;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonOp;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.lapeerftcrobotics.log.FileLogger;

/**
 * Created by User on 10/19/2016.
 */
public class AutonBaseImpl implements AutonStateMachineInterface {

    protected int state = -1;
    protected int stateCnt = 0;

    protected AutonOp autonOp = null;
    protected RobotController robotController = null;
    protected ElapsedTime runtime = null;
    protected Telemetry telemetry;
    protected ImageProcessingManager imageProcessingManager;
    protected FileLogger fileLogger;

    public AutonBaseImpl() {
    }

    public void setAutonOp(AutonOp a) {
        this.autonOp = a;
    }

    @Override
    public void setRobotController(RobotController r) {
        this.robotController = r;
    }

    @Override
    public void setRuntime(ElapsedTime r) {
        this.runtime = r;
    }

    @Override
    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    @Override
    public void setFileLogger(FileLogger f) {
        fileLogger = f;
    }

    @Override
    public void setImageProcessingManager(ImageProcessingManager imageProcessingManager) {
        this.imageProcessingManager = imageProcessingManager;
    }

    public int processStates(int state) {
        int nextState = state;
        return nextState;
    }

    public void process() {

        // 30ms each stateCnt
        int nextState = processStates(this.state);

        if (nextState != this.state) {
            this.state = nextState;
            this.stateCnt = 0;
        }
        else {
            this.stateCnt++;
        }
    }

    public int getState() {
        return this.state;
    }

}
