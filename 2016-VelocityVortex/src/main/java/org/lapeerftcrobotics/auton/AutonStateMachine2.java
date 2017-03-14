package org.lapeerftcrobotics.auton;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonOp;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.lapeerftcrobotics.log.FileLogger;

/**
 * Created by User on 11/10/2016.
 */
public class AutonStateMachine2 implements AutonStateMachineInterface {
    @Override
    public void process() {
        //Devin Jones was here! Just saying, code is one of the hardest parts of Robotics, aside from the hours-long car drives to competitions. Props to all who can even make a program! :D 11/10/16
    }

    @Override
    public int getState() {
        return 0;
    }

    @Override
    public void setAutonOp(AutonOp a) {

    }

    @Override
    public void setRobotController(RobotController r) {

    }

    @Override
    public void setRuntime(ElapsedTime runtime) {

    }

    @Override
    public void setTelemetry(Telemetry telemetry) {

    }

    @Override
    public void setImageProcessingManager(ImageProcessingManager imageProcessingManager) {

    }

    @Override
    public void setFileLogger(FileLogger fileLogger) {

    }
}
