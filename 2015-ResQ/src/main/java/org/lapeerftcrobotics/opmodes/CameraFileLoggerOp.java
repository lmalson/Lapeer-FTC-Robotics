package org.lapeerftcrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.camera.CameraHelper;
import org.lapeerftcrobotics.camera.ImageHandler;
import org.lapeerftcrobotics.camera.OpenCVCameraViewListener;
import org.lapeerftcrobotics.control.ControlLoopThread;
import org.lapeerftcrobotics.log.FileLogger;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class CameraFileLoggerOp extends OpMode {

  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  private long cnt = 0;
  private FileLogger fileLogger;

//  private ImageHandler imageHandler;

  private ControlLoopThread controlLoopThread = new ControlLoopThread();

  long prevLoopTime = 0;

  @Override
  public void init() {
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    cnt++;

      telemetry.addData("1","FileLogger Op Init Loop cnt: "+cnt+", ");
  }

  @Override
  public void start() {
      startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
      runtime.reset();
      telemetry.addData("FileLogger Op Start: ", runtime.toString());

      fileLogger = new FileLogger(runtime);
      fileLogger.open();
      telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());

      fileLogger.write("Time,SysMS,Thread,Event,Desc");

      fileLogger.writeEvent("start()","cnt: " + cnt);

      OpenCVCameraViewListener.getInstance().setFileLogger(fileLogger);
//      CameraHelper.getInstance().setFileLogger(fileLogger);
      controlLoopThread.setFileLogger(fileLogger);
      controlLoopThread.setRunning(true);
      controlLoopThread.start();
  }

  @Override
  public void stop() {
      telemetry.addData("FileLogger Op Stop: ", runtime.toString());
      OpenCVCameraViewListener.getInstance().setFileLogger(null);
      if (fileLogger != null) {
          fileLogger.writeEvent("stop()","cnt: " + cnt);
          fileLogger.close();
          fileLogger = null;
      }
      if (controlLoopThread != null) {
          controlLoopThread.setRunning(false);
          try {
              controlLoopThread.join();
          } catch (InterruptedException ex) {
          }
      }
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("1 Start", "FileLoggerOp started at " + startDate);
    telemetry.addData("2 Status", "running for " + runtime.toString());
    cnt++;

    long loopTime = System.currentTimeMillis();
    long loopPeriod = loopTime - prevLoopTime;
    prevLoopTime = loopTime;

    fileLogger.writeEvent("loop()","cnt: " + cnt+" loopPeriod: "+loopPeriod);

  }
}
