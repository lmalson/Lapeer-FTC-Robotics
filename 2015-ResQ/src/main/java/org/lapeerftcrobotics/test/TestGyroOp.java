package org.lapeerftcrobotics.test;

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
public class TestGyroOp extends OpMode {

  public static final String AUTON_VERSION = "2.0";
  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  private FileLogger fileLogger;
  private HiTechnicGyroSensor gyroSensor;
  private long lastTime = 0;
  private GyroSensor gs = null;


    public TestGyroOp() {
      super();
  }

  @Override
  public void init() {

      startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
      fileLogger = new FileLogger(runtime);
      fileLogger.open();
      fileLogger.write("Time,SysMS,Thread,Event,Desc");
      fileLogger.writeEvent("init()", "Version: " + AUTON_VERSION);
      telemetry.addData("D1", "Test Gyro init File: " + fileLogger.getFilename());

      gs = hardwareMap.gyroSensor.get("gyro");
      gyroSensor = new HiTechnicGyroSensor();
      gyroSensor.setFileLogger(fileLogger);
      gyroSensor.setGyro(gs);
      gyroSensor.setRunning(true);
      gyroSensor.start();
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

      telemetry.addData("Text", "*** Robot Started ***");
  }

  @Override
  public void stop() {
      fileLogger.writeEvent("stop()","STOP...");
      fileLogger.close();

      if (gyroSensor != null) {
          gyroSensor.setRunning(false);
          try {
              gyroSensor.join();
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
      long time = System.currentTimeMillis();
      telemetry.addData("G", "GS Rot: " + gs.getRotation());
      telemetry.addData("G2", "GS Head: "+gyroSensor.getHeadingRaw());

      if ((time-lastTime) > 400) {

          telemetry.addData("Gyro", "Heading: "+gyroSensor.getHeading());
          telemetry.addData("Gyro2", "Zero: "+gyroSensor.getOffset());
          telemetry.addData("Gyro3", "Dead: "+gyroSensor.getDeadBand());
          telemetry.addData("Gyro4", "State: " + gyroSensor.getGyroState());
          fileLogger.writeEvent("loop()", "heading: " + gyroSensor.getHeading() + " offset: " + gyroSensor.getOffset()+" deadband: "+gyroSensor.getDeadBand()+" state: "+gyroSensor.getState());
          lastTime = time;
      }
  }

}
