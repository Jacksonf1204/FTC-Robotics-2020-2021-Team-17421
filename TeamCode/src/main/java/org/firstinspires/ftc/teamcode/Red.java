/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red", group="Auto")
@Disabled
public class Red extends AutoCore {

    String size = "";

    int stopFlag = 0;



    @Override
    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        super.runOpMode();

        initAll();

        waitForStart();

        while(opModeIsActive() && stopFlag == 0) {

            // Scanning Starter Stack
            moveback(0.25,12);
            camRight();
            size = scanTarget();
            telemetry.addData("Scanned", size);
            telemetry.update();

            // Shooting Power Shots
            moveback(0.45, 42);

            shooterOn(0);
            sleep(2250); // was 1500 --> Then 2000
            flipForward();
            sleep(500);
            flipBackward();

            turn(0.45,-6);
            shooterOn(7);
            sleep(1125); // was 750 --> then 1000
            flipForward();
            sleep(200);
            flipBackward();

            turn(0.45,-12);
            shooterOn(7);
            sleep(1000);
            flipForward();
            sleep(200);
            flipBackward();
            shooterOff();

            sleep(200);
            turn(0.45,0);

            // Wobble Goal Placement
            if (size == "Quad")
            {
                moveback(0.25,48);
                turn(0.40,90);
                moveback(0.25,12);
                turn(0.40,175);
                // Drop Arm
                armDown();
                handOpen();
                sleep(200);
                moveback(0.45,36); // Was 6
                handClose();
                sleep(100);
                armUp();
                //moveback(0.45,30);
            }
            else if(size == "Single")
            {
                moveback(0.45,24);
                turn(0.40,90);
                moveforward(0.45,2);
                turn(0.40,175);
                // Drop Arm
                armDown();
                handOpen();
                sleep(200);
                moveback(0.45,6);
                handClose();
                sleep(100);
                armUp();
                moveback(0.45,8);
            }
            else
            {
                moveback(0.45,6);
                turn(0.40,90);
                moveback(0.25,15);
                turn(0.40,175);
                // Drop Arm
                armDown();
                handOpen();
                sleep(200);
                moveback(0.45,6);
                handClose();
                sleep(100);
                armUp();
                moveforward(0.45,8);
            }
            telemetry.addData("Finished", size);
            sleep(500);
            stopFlag = 1;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(-20, -20, 40, 40);
        dashboard.sendTelemetryPacket(packet);
    }

}