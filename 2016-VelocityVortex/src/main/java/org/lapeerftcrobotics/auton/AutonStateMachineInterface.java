package org.lapeerftcrobotics.auton;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonOp;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.lapeerftcrobotics.log.FileLogger;

/**
 * Created by User on 10/14/2016.
 */
public interface AutonStateMachineInterface {

    public void process();

    public int getState();

    public void setAutonOp(AutonOp a);
    
    public void setRobotController(RobotController r);

    public void setRuntime(ElapsedTime runtime);

    void setTelemetry(Telemetry telemetry);

    void setImageProcessingManager(ImageProcessingManager imageProcessingManager);

    void setFileLogger(FileLogger fileLogger);
}
