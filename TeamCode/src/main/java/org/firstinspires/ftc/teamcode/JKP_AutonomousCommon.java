/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.os.SystemClock;
/**
 *  Baseline Autonomous.  Common Stuff Goes Here!
 */

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class JKP_AutonomousCommon extends LinearOpMode {
    final double frameRate = 19;
    final double sensorFrameRate = 21;
    final double telemetryFrameRate = 1000;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    //A Timing System By Jeffrey & Alexis
    // long currentThreadTimeMillis (0);
    //
    final long SENSORPERIOD= 30;
    final long ENCODERPERIOD = 25;
    final long SERVOPERIOD = 35;
    final long NAVPERIOD = 35;
    final long MOTORPERIOD = 35;
    final long TELEMETRYPERIOD = 1000;

    static long CurrentTime=System.currentTimeMillis();
    static long LastSensor = CurrentTime;
    static long LastEncoderRead = CurrentTime;
    static long LastServo = CurrentTime;
    static long LastNav = CurrentTime;
    static long LastMotor = CurrentTime;
    static long LastTelemetry = CurrentTime;

    int overRun1 = 0;
    int overRun2 = 0;
    int skipped0 = 0;
    int skipped1 = 0;
    int skipped2 = 0;
    int accumTurn = 0;

    double currTime = System.currentTimeMillis();
    double legTime = currTime;
    double lastTelemetry = currTime;
    double timeLeft = 0;
    /***************************************************************************
     *            Everything below here happens after we press START           *
     ***************************************************************************/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * Put in calls to functions to initialize. Need to think about how to organize this.
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();  
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;

            }
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
            }
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;

            }
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;

            }
            if (CurrentTime - LastMotor > MOTORPERIOD){
                LastMotor = CurrentTime;
            }
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;

                telemetry.update();
            }

        }
    }
}
