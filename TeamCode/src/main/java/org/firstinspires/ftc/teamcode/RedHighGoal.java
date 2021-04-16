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


@Autonomous(name="RedHighGoal", group="Auto")
@Disabled
public class RedHighGoal extends AutoCore {

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
            // TODO: Set camera position here----------------------------------------
            size = scanTarget();
            telemetry.addData("Scanned", size);
            telemetry.update();

            // Shooting Power Shots
            moveback(0.45, 42);
            moveright(0.20,12);
            turn(0.35,0);

            shooterOn(42);
            sleep(2000); // was 1500 --> Then 2000
            flipForward();
            sleep(500);
            flipBackward();
            sleep(500); // was 750 --> then 1000
            flipForward();
            sleep(500);
            flipBackward();
            sleep(1000);
            flipForward();
            sleep(500);
            flipBackward();
            shooterOff();

            sleep(200);

            // Wobble Goal Placement

            if (size == "Quad")
            {
                moveback(0.25,48);
                turn(0.40,90);
                moveback(0.25,4);
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
                moveback(0.45,12);
                turn(0.45,90);
                // Drop Arm
                armDown();
                handOpen();
                sleep(200);
                moveback(0.45,6);
                handClose();
                sleep(100);
                armUp();
                turn(0.45,180);
                moveback(0.45,8);
            }
            else
            {
                moveback(0.45,6);
                turn(0.40,90);
                moveforward(0.25,2);
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