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

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

@TeleOp(name="RR Driver Control", group="Iterative Opmode")
//@Disabled
public class RRDriverControl extends RRRobotCore
{

    SampleMecanumDrive drive;

    public ElapsedTime powerShotTime = new ElapsedTime();

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
    boolean powerShot = false;

    public ElapsedTime shotTime2 = new ElapsedTime();
    int number2 = 1;

    enum FlipShoot{
        SHOT_ONE,
        SHOT_TWO,
        SHOT_THREE,
        FINISH_THREE,
        IDLE
    }


    enum StateShoot{
        FLIP_FORWARD,
        FLIP_BACKWARD,
        IDLE
    }

    Trajectory powerShot1;
    Trajectory powerShot2;
    Trajectory powerShot3;

    FlipShoot flipperShoot = FlipShoot.IDLE;
    StateShoot stateShoot = StateShoot.IDLE;

    Pose2d startPose2 = new Pose2d(-12, 18, Math.toRadians(180));
    @Override
    public void init() {

        super.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose2);


        powerShot1 = drive.trajectoryBuilder(startPose2, false)
                .lineTo(new Vector2d(-12,-2))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    // Turn on the shooter here
                    shooterOn(-76);
                })
                .build();

        powerShot2 = drive.trajectoryBuilder(powerShot1.end(), false)
                .lineTo(new Vector2d(-12,-9))
                .build();

        powerShot3 = drive.trajectoryBuilder(powerShot2.end(), false)
                .lineTo(new Vector2d(-12,-16))
                .build();

        flipperShoot = FlipShoot.SHOT_ONE;
        stateShoot = StateShoot.FLIP_FORWARD;

    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.update();

        x1 = gamepad1.left_stick_x;
        x2 = gamepad1.right_stick_x;
        y1 = -gamepad1.left_stick_y;

        // Motor Power Values After X and Y Axis Calculated
        rfp = (y1 - x2 - x1);
        rrp = (y1 - x2 + x1);
        lfp = (y1 + x2 + x1);
        lrp = (y1 + x2 - x1);

            if (powerShot == false) {

                // Add more power to the motor after the input is calculated to make it faster
                if (rfp < 1 && rfp > 0) {
                    if (rfp + 0.2 > 1) {
                        rfp = 1;
                    } else {
                        rfp += 0.2;
                    }
                } else if (rfp > -1 && rfp < 0) {
                    if (rfp - 0.2 < -1) {
                        rfp = -1;
                    } else {
                        rfp -= 0.2;
                    }
                }

                if (rrp < 1 && rrp > 0) {
                    if (rrp + 0.2 > 1) {
                        rrp = 1;
                    } else {
                        rrp += 0.2;
                    }
                } else if (rrp > -1 && rrp < 0) {
                    if (rrp - 0.2 < -1) {
                        rrp = -1;
                    } else {
                        rrp -= 0.2;
                    }
                }

                if (lfp < 1 && lfp > 0) {
                    if (lfp + 0.2 > 1) {
                        lfp = 1;
                    } else {
                        lfp += 0.2;
                    }
                } else if (lfp > -1 && lfp < 0) {
                    if (lfp - 0.2 < -1) {
                        lfp = -1;
                    } else {
                        lfp -= 0.2;
                    }
                }

                if (lrp < 1 && lrp > 0) {
                    if (lrp + 0.2 > 1) {
                        lrp = 1;
                    } else {
                        lrp += 0.2;
                    }
                } else if (lrp > -1 && lrp < 0) {
                    if (lrp - 0.2 < -1) {
                        lrp = -1;
                    } else {
                        lrp -= 0.2;
                    }
                }


                // Driving Controls

                if (gamepad1.left_stick_x >= threshold || gamepad1.left_stick_x <= -threshold
                        || gamepad1.left_stick_y >= threshold || gamepad1.left_stick_y <= -threshold
                        || gamepad1.right_stick_x >= threshold || gamepad1.right_stick_x <= -threshold) {

                    rightFront.setPower(rfp * (1 - gamepad1.left_trigger));
                    rightRear.setPower(rrp * (1 - gamepad1.left_trigger));
                    leftFront.setPower(lfp * (1 - gamepad1.left_trigger));
                    leftRear.setPower(lrp * (1 - gamepad1.left_trigger));
                } else {
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                }


                // Intake Control

                if (gamepad1.right_bumper) {
                    intake.setPower(-1);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(1);
                } else if (gamepad1.right_trigger > 0.7) {
                    intake.setPower(0);
                }

                // Arm Control

                if (gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05) {
                    arm.setPower(gamepad2.left_stick_y * 0.6);
                } else {
                    arm.setPower(0);
                }

                // Shooting Controls
                if (gamepad2.right_bumper) {
                    shooter1.setVelocity(685);
                    shooter2.setVelocity(685);
                } else if (gamepad2.left_bumper) {
                    shooter1.setVelocity(730); // Was 744 on 3/8/21
                    shooter2.setVelocity(730); // Was 744 on 3/8/21
                } else if (gamepad2.left_trigger >= 0.4) {
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                }

                // Hand Servo
                if (gamepad2.b) {
                    handClose();
                } else if (gamepad2.x) {
                    handOpen();
                }

                if (gamepad2.right_trigger > 0.2) {
                    flipForward();
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
                } else {
                    flipBackward();
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("GOLD"));
                }

                if (gamepad1.dpad_up) {
                    powerShot = true;
                    flipperShoot = FlipShoot.SHOT_ONE;
                    stateShoot = StateShoot.FLIP_FORWARD;
                    number2 = 1;
                    drive.setPoseEstimate(startPose2);
                }

            }
            if (powerShot == true) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("AQUA"));
                arm.setPower(0);
                telemetry.addLine("powerShot True");
                telemetry.update();
                switch (flipperShoot) {
                    case SHOT_ONE:
                        telemetry.addLine("Shot One");
                        telemetry.update();
                        if (gamepad1.dpad_down)
                        {
                            flipperShoot = FlipShoot.IDLE;
                            stateShoot = StateShoot.FLIP_FORWARD;
                            number2 = 1;
                            powerShot = false;
                        }
                        // Check if the drive class isn't busy
                        // After !isBusy() == true, we move to next state
                        // Make sure to use async follow function
                        //if (!drive.isBusy()) {
                            flipperShoot = FlipShoot.SHOT_TWO;
                            drive.followTrajectoryAsync(powerShot1);
                        //}
                        break;
                    case SHOT_TWO:
                        if (gamepad1.dpad_down)
                        {

                            flipperShoot = FlipShoot.IDLE;
                            stateShoot = StateShoot.FLIP_FORWARD;
                            number2 = 1;
                            powerShot = false;
                        }
                        // Check if the drive class isn't busy
                        // After !isBusy() == true, we move to next state
                        // Make sure to use async follow function
                        if (!drive.isBusy()) {
                            if (number2 == 1) {
                                shotTime2.reset();
                                number2 = 2;
                            }
                            switch (stateShoot) {
                                case FLIP_FORWARD:
                                    flipForward();
                                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.FLIP_BACKWARD;
                                        shotTime2.reset();
                                    }
                                    break;
                                case FLIP_BACKWARD:
                                    flipBackward();
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.IDLE;
                                        shotTime2.reset();
                                    }
                                    break;
                                case IDLE:
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                    number2 = 1;
                                    flipperShoot = FlipShoot.SHOT_THREE;
                                    drive.followTrajectoryAsync(powerShot2);
                                    break;
                            }
                        }
                        break;
                    case SHOT_THREE:
                        if (gamepad1.dpad_down)
                        {
                            flipperShoot = FlipShoot.IDLE;
                            stateShoot = StateShoot.FLIP_FORWARD;
                            number2 = 1;
                            powerShot = false;
                        }
                        // Check if the drive class isn't busy
                        // After !isBusy() == true, we move to next state
                        // Make sure to use async follow function
                        if (!drive.isBusy()) {
                            if (number2 == 1) {
                                shotTime2.reset();
                                number2 = 2;
                            }
                            switch (stateShoot) {
                                case FLIP_FORWARD:
                                    flipForward();
                                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.FLIP_BACKWARD;
                                        shotTime2.reset();
                                    }
                                    break;
                                case FLIP_BACKWARD:
                                    flipBackward();
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.IDLE;
                                        shotTime2.reset();
                                    }
                                    break;
                                case IDLE:
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                    number2 = 1;
                                    flipperShoot = FlipShoot.FINISH_THREE;
                                    drive.followTrajectoryAsync(powerShot3);
                                    break;
                            }
                        }
                        break;
                    case FINISH_THREE:
                        if (gamepad1.dpad_down)
                        {
                            flipperShoot = FlipShoot.IDLE;
                            stateShoot = StateShoot.FLIP_FORWARD;
                            number2 = 1;
                            powerShot = false;
                        }
                        if (!drive.isBusy()) {
                            if (number2 == 1) {
                                shotTime2.reset();
                                number2 = 2;
                            }
                            switch (stateShoot) {
                                case FLIP_FORWARD:
                                    flipForward();
                                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.FLIP_BACKWARD;
                                        shotTime2.reset();
                                    }
                                    break;
                                case FLIP_BACKWARD:
                                    flipBackward();
                                    if (shotTime2.time() > 0.35) {
                                        stateShoot = StateShoot.IDLE;
                                        shotTime2.reset();
                                    }
                                    break;
                                case IDLE:
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                    number2 = 1;
                                    flipperShoot = FlipShoot.IDLE;
                                    break;
                            }
                        }
                        break;
                    case IDLE:
                        powerShot = false;
                        break;
                    }
            }


    }


}
