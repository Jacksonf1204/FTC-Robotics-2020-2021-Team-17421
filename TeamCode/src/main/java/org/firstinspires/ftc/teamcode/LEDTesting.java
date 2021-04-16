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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

@TeleOp(name="LED Testing", group="Testing")
//@Disabled
public class LEDTesting extends OpMode {

    RevBlinkinLedDriver leds;

    @Override
    public void init() {

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("GOLD"));
        }
        else if (gamepad1.a)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
        }
        else if (gamepad1.b)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("LIGHT_CHASE_RED"));
        }
        else if (gamepad1.x)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("HEARTBEAT_RED"));
        }
        else if (gamepad1.y)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("BREATH_RED"));
        }
        else if (gamepad1.left_bumper)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("STROBE_RED"));
        }
        else if (gamepad1.dpad_up)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("LARSON_SCANNER_RED"));
        }
        else if (gamepad1.dpad_down)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("FIRE_LARGE"));
        }
        else if (gamepad1.dpad_right)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("SINELON_LAVA_PALETTE"));
        }
        else if (gamepad1.dpad_left)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("CP1_SHOT"));
        }
        else if (gamepad1.left_trigger > 0.2)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("BEATS_PER_MINUTE_LAVA_PALETTE"));
        }
        else if (gamepad1.right_trigger > 0.2)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RAINBOW_LAVA_PALETTE"));
        }
    }

}