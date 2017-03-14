/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.lapeerftcrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lapeerftcrobotics.log.FileLogger;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class FileLoggerOp extends OpMode {

  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  private long cnt = 0;
  private FileLogger fileLogger;

  @Override
  public void init() {
      fileLogger = new FileLogger(runtime);
      fileLogger.open();
      telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());

      fileLogger.write("Time,Event,Desc");
      fileLogger.write(runtime.toString()+","+"init()"+","+"N/A");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    cnt++;
    telemetry.addData("FileLogger Op Init Loop cnt: ",cnt);

    fileLogger.write(runtime.toString() + "," + "init_loop()" + "," + "cnt: " + cnt);

  }

  @Override
  public void start() {
      startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
      runtime.reset();
      telemetry.addData("FileLogger Op Start: ", runtime.toString());
      fileLogger.write(runtime.toString() + "," + "start()" + "," + "cnt: " + cnt);
  }

  @Override
  public void stop() {
      telemetry.addData("FileLogger Op Stop: ", runtime.toString());
      fileLogger.write(runtime.toString() + "," + "stop()" + "," + "cnt: " + cnt);
      fileLogger.close();
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
    fileLogger.write(runtime.toString()+","+"loop()"+","+"cnt: "+cnt);
  }
}
