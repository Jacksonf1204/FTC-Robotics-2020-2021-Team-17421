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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="FlipperTest", group="Testing")
@Disabled
public class ShoulderTest extends OpMode {

    DcMotorEx shooter1 = null;
    DcMotorEx shooter2 = null;
    Servo flipper = null;
    double flipPos = 0.0;

    @Override
    public void init() {

        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        flipper = hardwareMap.get(Servo.class, "flipper");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setVelocityPIDFCoefficients(225,5,70,15); // 15 3 5 0 top with 300 p
        shooter2.setVelocityPIDFCoefficients(225,5,70,15); // 15 3 5 0 top with 300 p
    }

    // Code gets run 360 times per second
    @Override
    public void loop() {

        if (gamepad2.left_bumper) {
            shooter1.setVelocity(772);
            shooter2.setVelocity(772);// Was 744
        } else if (gamepad2.left_trigger > 0.5) {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        if (gamepad2.right_trigger > 0.4) {
            flipper.setPosition(0.13);
        } else {
            flipper.setPosition(0.02);
        }


        if (gamepad2.dpad_right){
            flipPos = flipper.getPosition();
            flipper.setPosition(flipPos + 0.01);
        }
        else if (gamepad2.dpad_left){
            flipPos = flipper.getPosition();
            flipper.setPosition(flipPos - 0.01);
        }
        telemetry.addData("Pos",flipper.getPosition());
        telemetry.update();


    }


}