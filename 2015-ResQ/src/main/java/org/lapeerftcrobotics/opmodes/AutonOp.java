package org.lapeerftcrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.auton.AutonStateMachine;
import org.lapeerftcrobotics.auton.Beacon;
import org.lapeerftcrobotics.auton.Ramp;
import org.lapeerftcrobotics.camera.OpenCVCameraViewListener;
import org.lapeerftcrobotics.control.Alliance;
import org.lapeerftcrobotics.control.BeaconTargetController;
import org.lapeerftcrobotics.control.ControlLoopThread;
import org.lapeerftcrobotics.control.DriverControls;
import org.lapeerftcrobotics.control.HiTechnicGyroSensor;
import org.lapeerftcrobotics.control.RampTargetController;
import org.lapeerftcrobotics.control.RobotController;
import org.lapeerftcrobotics.log.FileLogger;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class AutonOp extends OpMode {

  public static final String AUTON_VERSION = "1.0";
  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  private FileLogger fileLogger;

  private Alliance alliance;
  private int autonMode;
  private boolean delay = false;
  public final static int AUTON_BLUE_MODE = 0;
  public final static int AUTON_BLUE_DELAY_MODE = 1;
  public final static int AUTON_RED_MODE = 2;
  public final static int AUTON_RED_DELAY_MODE  = 3;

  private ControlLoopThread controlLoopThread = null;
  private Beacon beacon = null;
  private BeaconTargetController beaconTargetController = null;
  private AutonStateMachine autonStateMachine = null;
  private RobotController robotController = null;
  private Ramp ramp = null;
  private RampTargetController rampTargetController = null;
  private GyroSensor gyroSensor = null;

  public AutonOp() {
      super();
  }

  public void setAutonMode(int am) {
      this.autonMode = am;
      switch(this.autonMode) {
          case AUTON_BLUE_MODE: {
              this.delay = false;
              this.alliance = new Alliance(Alliance.BLUE_ALLIANCE);
              break;
          }
          case AUTON_BLUE_DELAY_MODE: {
              this.delay = true;
              this.alliance = new Alliance(Alliance.BLUE_ALLIANCE);
              break;
          }
          case AUTON_RED_MODE: {
              this.delay = false;
              this.alliance = new Alliance(Alliance.RED_ALLIANCE);
              break;
          }
          case AUTON_RED_DELAY_MODE: {
              this.delay = true;
              this.alliance = new Alliance(Alliance.RED_ALLIANCE);
              break;
          }
      }
  }

  @Override
  public void init() {
      beacon = new Beacon();
      beaconTargetController = new BeaconTargetController(this.alliance);
      beaconTargetController.setBeacon(beacon);
      ramp = new Ramp();
      rampTargetController = new RampTargetController(this.alliance);
      rampTargetController.setRamp(ramp);
      robotController = new RobotController();
      robotController.setLeftDriveMotor(hardwareMap.dcMotor.get("motor_1"));
      robotController.setRightDriveMotor(hardwareMap.dcMotor.get("motor_2"));
      robotController.setWinchMotor(hardwareMap.dcMotor.get("motor_3"));
      robotController.setArmRotateMotor(hardwareMap.dcMotor.get("motor_4"));

      robotController.setRightAllClearArmServo(hardwareMap.servo.get("servo_1"));
      robotController.setLeftAllClearArmServo(hardwareMap.servo.get("servo_2"));
      robotController.setWinchAngleServo(hardwareMap.servo.get("servo_3"));
      robotController.setRotateServo(hardwareMap.servo.get("servo_4"));
      robotController.setArmSlideServo(hardwareMap.servo.get("servo_5"));
      robotController.setRightGrabberServo(hardwareMap.servo.get("servo_6"));
      robotController.setArmAngleServo(hardwareMap.servo.get("servo_8"));
      robotController.init();

      gyroSensor = hardwareMap.gyroSensor.get("gyro");

      // calibrate the gyro.
      gyroSensor.calibrate();
      robotController.setGyroSensor(gyroSensor);

      autonStateMachine = new AutonStateMachine(this.alliance,this);
      autonStateMachine.setBeacon(beacon);
      autonStateMachine.setBeaconTargetController(beaconTargetController);
      autonStateMachine.setRamp(ramp);
      autonStateMachine.setRampTargetController(rampTargetController);
      autonStateMachine.setRobotController(robotController);
      autonStateMachine.setDelay(delay);

      controlLoopThread = new ControlLoopThread();
      controlLoopThread.setAutonStateMachine(autonStateMachine);
      controlLoopThread.setBeaconTargetController(beaconTargetController);
      controlLoopThread.setRampTargetController(rampTargetController);
      controlLoopThread.setRobotController(robotController);

      startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
      fileLogger = new FileLogger(runtime);
      fileLogger.open();
      fileLogger.write("Time,SysMS,Thread,Event,Desc");
      fileLogger.writeEvent("init()", "Version: " + AUTON_VERSION + " alliance blue: " + alliance.isBlueAlliance());

      autonStateMachine.setFileLogger(fileLogger);
      controlLoopThread.setFileLogger(fileLogger);
      robotController.setFileLogger(fileLogger);
      OpenCVCameraViewListener.getInstance().setElapsedTime(runtime);
      OpenCVCameraViewListener.getInstance().setFileLogger(fileLogger);
      OpenCVCameraViewListener.getInstance().setAlliance(alliance);
      OpenCVCameraViewListener.getInstance().setBeacon(beacon);
      OpenCVCameraViewListener.getInstance().setBeaconTargetController(beaconTargetController);
      OpenCVCameraViewListener.getInstance().setAutonStateMachine(autonStateMachine);
      OpenCVCameraViewListener.getInstance().setRobotController(robotController);
      OpenCVCameraViewListener.getInstance().setState(OpenCVCameraViewListener.NOOP_STATE);

      telemetry.addData("FileLogger Op Init: ", runtime.toString()+" File: "+fileLogger.getFilename());
      if (alliance.isBlueAlliance())
          telemetry.addData("All", "*** BLUE Alliance ***");
      else
          telemetry.addData("All", "*** RED Alliance ***");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
  }

  @Override
  public void start() {
      runtime.reset();

      autonStateMachine.start();
      controlLoopThread.setRunning(true);
      controlLoopThread.start();

      telemetry.addData("Text", "*** Robot Started ***");
  }

  @Override
  public void stop() {
      fileLogger.writeEvent("stop()","STOP...");
      fileLogger.close();

      if (controlLoopThread != null) {
          controlLoopThread.setRunning(false);
          try {
              controlLoopThread.join();
          } catch (InterruptedException ex) {
          }
      }
      telemetry.addData("Stop", "*** Robot Stopped ***  "+runtime.toString());
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
      telemetry.addData("Auton", "Auton State: "+autonStateMachine.getState()+" Cnt: "+autonStateMachine.getStateCnt());
      telemetry.addData("CV", "OpenCV State: "+OpenCVCameraViewListener.getInstance().getState());
  }


}
