package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachine1_7138;
import org.lapeerftcrobotics.auton.AutonStateMachine1_8935;
import org.lapeerftcrobotics.auton.AutonStateMachine2_7138;
import org.lapeerftcrobotics.auton.AutonStateMachine2_8935;
import org.lapeerftcrobotics.auton.AutonStateMachineInterface;
import org.lapeerftcrobotics.control.ControlLoopThread;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.imageprocess.ImageProcessingManager;
import org.lapeerftcrobotics.log.FileLogger;
import org.lapeerftcrobotics.vision.OnCameraFrameCallbackInterface;
import org.lapeerftcrobotics.vision.OpenCVCameraActivityHelper;
import org.lapeerftcrobotics.vision.OpenCVCameraViewListener;
import org.opencv.core.Mat;

/**
 * Created by User on 10/14/2016.
 */
public class AutonOp extends OpMode implements OnCameraFrameCallbackInterface {

    public final static char BLUE_ALLIANCE = 'B';
    public final static char RED_ALLIANCE = 'R';

    public final static int AUTON_MODE_1_8935 = 1;
    public final static int AUTON_MODE_1_7138 = 2;
    public final static int AUTON_MODE_2_8935 =3;
    public final static int AUTON_MODE_2_7138 = 4;

    private char alliance = ' ';
    private int autonMode = 0;

    private AutonStateMachineInterface autonStateMachine = null;
    private RobotController robotController = null;
    private ControlLoopThread controlLoopThread = null;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCVCameraActivityHelper openCVCameraActivityHelper = null;
    private ImageProcessingManager imageProcessingManager = null;
    private FileLogger fileLogger;

    public void setAlliance(char a) {
        this.alliance = a;
    }

    public void setAutonMode(int m) {
        this.autonMode = m;
    }

    public boolean isBlueAlliance() {
        if (this.alliance == BLUE_ALLIANCE)
            return true;
        else
            return false;
    }

    public boolean isRedAlliance() {
        if (this.alliance == RED_ALLIANCE)
            return true;
        else
            return false;
    }

    public ElapsedTime getTime() {
        return runtime;
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        openCVCameraActivityHelper = new OpenCVCameraActivityHelper();
        openCVCameraActivityHelper.init();

        robotController = new RobotController();
        robotController.init(hardwareMap);
        if (autonMode == AUTON_MODE_1_8935) {
            robotController.setRobot(RobotController.ROBOT_8935);
        }
        else if (autonMode == AUTON_MODE_1_7138) {
            robotController.setRobot(RobotController.ROBOT_7138);
        }
        else if (autonMode == AUTON_MODE_2_8935) {
            robotController.setRobot(RobotController.ROBOT_8935);
        }
        else if (autonMode == AUTON_MODE_2_7138) {
            robotController.setRobot(RobotController.ROBOT_7138);
        }

        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());

        telemetry.addData("Status", "Initialized");
    }

    /*
 * Code to run ONCE when the driver hits PLAY
 */
    @Override
    public void start() {

        OpenCVCameraViewListener.getInstance().registerOnFrameCallbackListener(this);

        if (autonMode == AUTON_MODE_1_8935) {
            autonStateMachine = new AutonStateMachine1_8935();
        }
        else if (autonMode == AUTON_MODE_1_7138) {
            autonStateMachine = new AutonStateMachine1_7138();
        }
        else if (autonMode == AUTON_MODE_2_8935) {
            autonStateMachine = new AutonStateMachine2_8935();
        }
        else if (autonMode == AUTON_MODE_2_7138) {
            autonStateMachine = new AutonStateMachine2_7138();
        }

        imageProcessingManager = new ImageProcessingManager();
        imageProcessingManager.setFileLogger(fileLogger);
        imageProcessingManager.setTelemetry(telemetry);
        imageProcessingManager.setAutonOp(this);

        autonStateMachine.setAutonOp(this);
        autonStateMachine.setRobotController(robotController);
        autonStateMachine.setRuntime(runtime);
        autonStateMachine.setTelemetry(telemetry);
        autonStateMachine.setImageProcessingManager(imageProcessingManager);
        autonStateMachine.setFileLogger(fileLogger);

        controlLoopThread = new ControlLoopThread();
        controlLoopThread.setAutonStateMachineInterface(autonStateMachine);
        controlLoopThread.setRobotController(robotController);
        controlLoopThread.setRunning(true);
        controlLoopThread.start();

        runtime.reset();

        telemetry.addData("Status", "Started");
    }

        @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        fileLogger.close();
        openCVCameraActivityHelper.stop();
        OpenCVCameraViewListener.getInstance().registerOnFrameCallbackListener(null);
        if (controlLoopThread != null) {
            controlLoopThread.setRunning(false);
            try {
                controlLoopThread.join();
            } catch (InterruptedException ex) {
            }
        }
    }

    /**
     * This method is called by the OpenCV Camera Thread independent of the FTC loop() method.
     * @param in
     * @return
     */
    @Override
    public Mat process(Mat in) {
//        Point center = new Point(240,160);
//        Imgproc.circle(in, center, 30, new Scalar(0, 255, 0), -1);
        return imageProcessingManager.process(in);
    }
}
