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
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//**************************************************************************************************
// Add gradual speed up to shooter
// Add intake one touch button
//**************************************************************************************************

@TeleOp(name="Driver Control", group="Iterative Opmode")
@Disabled
public class DriverControl extends RobotCore
{
    

    double power = 0.75;
    double variance = 0;

    // Values created from positions of the joysticks
    double x1 = 0;
    double x2 = 0;
    double y1 = 0;

    // Motor Power Values After X and Y Axis Calculated
    double rfp = 0;
    double rrp = 0;
    double lfp = 0;
    double lrp = 0;
    double armencoder = 0;



    @Override
    public void init() {

        super.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

            x1 = gamepad1.left_stick_x;
            x2 = gamepad1.right_stick_x;
            y1 = -gamepad1.left_stick_y;

            // Motor Power Values After X and Y Axis Calculated
            rfp = (y1 - x2 - x1);
            rrp = (y1 - x2 + x1);
            lfp = (y1 + x2 + x1);
            lrp = (y1 + x2 - x1);

            // Add more power to the motor after the input is calculated to make it faster
            if (rfp < 1 && rfp > 0)
            {
                if (rfp + 0.2 > 1){
                    rfp = 1;
                }
                else
                {
                    rfp += 0.2;
                }
            }
            else if(rfp >-1 && rfp < 0)
            {
                if (rfp - 0.2 < -1){
                    rfp = -1;
                }
                else
                {
                    rfp -= 0.2;
                }
            }

            if (rrp < 1 && rrp > 0)
            {
                if (rrp + 0.2 > 1){
                    rrp = 1;
                }
                else
                {
                    rrp += 0.2;
                }
            }
            else if(rrp >-1 && rrp < 0)
            {
                if (rrp - 0.2 < -1){
                    rrp = -1;
                }
                else
                {
                    rrp -= 0.2;
                }
            }

            if (lfp < 1 && lfp > 0)
            {
                if (lfp + 0.2 > 1){
                    lfp = 1;
                }
                else
                {
                    lfp += 0.2;
                }
            }
            else if(lfp >-1 && lfp < 0)
            {
                if (lfp - 0.2 < -1){
                    lfp = -1;
                }
                else
                {
                    lfp -= 0.2;
                }
            }

            if (lrp < 1 && lrp > 0)
            {
                if (lrp + 0.2 > 1){
                    lrp = 1;
                }
                else
                {
                    lrp += 0.2;
                }
            }
            else if(lrp >-1 && lrp < 0)
            {
                if (lrp - 0.2 < -1){
                    lrp = -1;
                }
                else
                {
                    lrp -= 0.2;
                }
            }





            // Driving Controls

            if (gamepad1.left_stick_x >= threshold || gamepad1.left_stick_x <= -threshold
                    || gamepad1.left_stick_y >= threshold || gamepad1.left_stick_y <= -threshold
                    || gamepad1.right_stick_x >= threshold || gamepad1.right_stick_x <= -threshold ){

                rightFront.setPower(rfp * (1 - gamepad1.left_trigger));
                rightRear.setPower(rrp * (1 - gamepad1.left_trigger));
                leftFront.setPower(lfp * (1 - gamepad1.left_trigger));
                leftRear.setPower(lrp * (1 - gamepad1.left_trigger));
            }
            else{
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
            }



            // Intake Control

            if (gamepad1.right_bumper){
                intake.setPower(-1);
            }
            else if (gamepad1.left_bumper){
                intake.setPower(1);
            }
            else if (gamepad1.right_trigger > 0.7){
                intake.setPower(0);
            }

            // Arm Control

            if (gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05)
            {
                arm.setPower(gamepad2.left_stick_y * 0.75);
            }
            else
            {
                arm.setPower(0);
            }

            // Shooting Controls

            if (gamepad2.right_bumper)
            {
                shooter1.setVelocity(709); // Was 828 -- Shot On Line
                shooter2.setVelocity(709); // Was 828 -- Shot On Line
            }
            else if (gamepad2.left_bumper)
            {
                shooter1.setVelocity(744); // Was 852 -- Shot On Line
                shooter2.setVelocity(744); // Was 852 -- Shot On Line
            }
            else if (gamepad2.left_trigger >= 0.4)
            {
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
            }

            // Hand Servo
            if(gamepad2.b)
            {
                handClose();
            }
            else if (gamepad2.x)
            {
                handOpen();
            }

            if (gamepad2.right_trigger > 0.2)
            {
                flipForward();
            }
            else
            {
                flipBackward();
            }

    }


}
