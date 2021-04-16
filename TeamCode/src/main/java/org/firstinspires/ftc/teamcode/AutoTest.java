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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name="RedOutside", group="Auto")
@Disabled
public class AutoTest extends AutoCore {

    DcMotor rightRear = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor leftFront = null;

    int stopFlag = 0;
    double encoderDistance = 7;


    @Override
    public void runOpMode(){

        rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Brakes motors when not given power
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();

        waitForStart();

        while(opModeIsActive() && stopFlag == 0) {

            stopFlag = 1;
        }
    }

    void resetEncoder() {
        // Starts motor encoders to track distance
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets motor encoders so we don't have to worry about previous movements
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // ***********************************************************************
    // MoveForward
    // ***********************************************************************
    // Causes robot to move forward
    // Used in autonomous
    void moveforward(double speed, double distance) {

        double angle = curHeading;
        // This is the inch to encoder distance
        distance *= encoderDistance;

        double finalspeed = 0;
        // Starts and Resets motor encoders
        resetEncoder();

        // This adjusts the power if the robot drifts off course
        double adjustRightPower = 1;
        double adjustLeftPower = 1;

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightFront.getCurrentPosition() < distance) {
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            if (curHeading > angle + 1)
            {
                adjustLeftPower = 0.5;
            }
            else if(curHeading < angle - 1)
            {
                adjustRightPower = 0.5;
            }
            else
            {
                adjustRightPower = 1;
                adjustLeftPower = 1;
            }
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() < (distance * 0.6)) {
                rightFront.setPower(speed * adjustRightPower);
                rightRear.setPower(speed * adjustRightPower);
                leftFront.setPower(speed * adjustLeftPower);
                leftRear.setPower(speed * adjustLeftPower);
            }
            // If the robot is past 60%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() < (distance)) {
                rightFront.setPower(speed * 0.5);
                rightRear.setPower(speed * 0.5);
                leftFront.setPower(speed * 0.5);
                leftRear.setPower(speed * 0.5);
            }
            //angleCorrection(curHeading);
        }

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        // Stops motor encoders
        stopEncoder();
    }

    // ***********************************************************************
    // MoveBack
    // ***********************************************************************
    // Causes robot to move backwards
    // Used in autonomous
    void moveback(double speed, double distance) {

        double angle = curHeading;
        // This is the inch to encoder distance
        distance *= encoderDistance;
        // This adjusts the power if the robot drifts off course
        double adjustRightPower = 1;
        double adjustLeftPower = 1;

        double finalspeed = 0;
        // Starts and Resets motor encoders
        resetEncoder();

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightFront.getCurrentPosition() > -distance) {
            telemetry.addData("Angle", curHeading);
            telemetry.update();
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() > (-distance * 0.9)) {
                if (curHeading > angle + 2)
                {
                    adjustLeftPower = 0.75;
                }
                else if(curHeading < angle - 2)
                {
                    adjustRightPower = 0.75;
                }
                else
                {
                    adjustRightPower = 1;
                    adjustLeftPower = 1;
                }
                telemetry.addData("Speed", finalspeed);
                telemetry.update();
                rightFront.setPower(-finalspeed * adjustRightPower);
                rightRear.setPower(-finalspeed * adjustRightPower);
                leftFront.setPower(-finalspeed * adjustLeftPower);
                leftRear.setPower(-finalspeed * adjustLeftPower);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() > (-distance)) {
                telemetry.addData("Speed", finalspeed);
                telemetry.update();
                rightFront.setPower(-finalspeed * 0.5);
                rightRear.setPower(-finalspeed * 0.5);
                leftFront.setPower(-finalspeed * 0.5);
                leftRear.setPower(-finalspeed * 0.5);
            }
            //angleCorrection(curHeading);
        }

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        // Stops motor encoders
        stopEncoder();

    }

    // ***********************************************************************
    // MoveLeft
    // ***********************************************************************
    // Causes robot to move left
    // Used in autonomous
    void moveleft(double speed, double distance) {

        double angle = curHeading + 360;
        // This is the inch to encoder distance
        distance *= encoderDistance;

        double finalspeed = 0;
        // Starts and Resets motor encoders
        resetEncoder();

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightFront.getCurrentPosition() < distance) {
            telemetry.addData("RightHeading", curHeading);
            telemetry.update();
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() < (distance * 0.9)) {
                rightFront.setPower(speed);
                rightRear.setPower(-speed);
                leftFront.setPower(-speed);
                leftRear.setPower(speed);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() < (distance)) {
                rightFront.setPower(speed * 0.5);
                rightRear.setPower(-speed * 0.5);
                leftFront.setPower(-speed * 0.5);
                leftRear.setPower(speed * 0.5);
            }
        }

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        // Stops motor encoders
        stopEncoder();

    }

    // ***********************************************************************
    // MoveRight
    // ***********************************************************************
    // Causes robot to move right
    // Used in autonomous
    void moveright(double speed, double distance) {

        checkOrientation();

        // This is the inch to encoder distance
        distance *= encoderDistance;


        double finalspeed = 0;
        // Starts and Resets motor encoders
        resetEncoder();

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightFront.getCurrentPosition() > -distance) {
            telemetry.addData("RightHeading", curHeading);
            telemetry.update();
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
            }
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() > (-distance * 0.9)) {
                rightFront.setPower(-speed);
                rightRear.setPower(speed);
                leftFront.setPower(speed);
                leftRear.setPower(-speed);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() > (-distance)) {
                rightFront.setPower(-speed * 0.5);
                rightRear.setPower(speed * 0.5);
                leftFront.setPower(speed * 0.5);
                leftRear.setPower(-speed * 0.5);
            }
        }

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        // Stops motor encoders
        stopEncoder();

    }

    // Void turn

    void turn(double speed, double angle)
    {
        double finalspeed = 0;
        double angleDistance = 0;
        double wrongAngle = 0;
        checkOrientation();
        angleDistance = (180 + curHeading) - angle;
        if ((angleDistance >= 180 && angleDistance <= 360) || angleDistance <= 0) // Between 180 and 360 or less than 0
        {
            while(angleDistance >= 182 || angleDistance <= 178) // Turning Left
            {
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                checkOrientation();
                telemetry.addData("Heading To Left", curHeading);
                angleDistance = (180 + curHeading) - angle;
                if (angleDistance >= 200 || angleDistance <= 160)
                {
                    rightFront.setPower(finalspeed);
                    rightRear.setPower(finalspeed);
                    leftFront.setPower(-finalspeed);
                    leftRear.setPower(-finalspeed);
                }
                else
                {
                    rightFront.setPower(finalspeed* 0.75);
                    rightRear.setPower(finalspeed * 0.75);
                    leftFront.setPower(-finalspeed * 0.75);
                    leftRear.setPower(-finalspeed * 0.75);
                }

                telemetry.update();
            }
        }
        else // Between 1 and 179 or greater than 360
        {
            while(angleDistance >= 182 || angleDistance <= 178) // Turning Right
            {
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                if (finalspeed < speed)
                {
                    finalspeed += 0.01;
                }
                checkOrientation();
                telemetry.addData("Heading To Right", curHeading);
                angleDistance = (180 + curHeading) - angle;
                if (angleDistance >= 200 || angleDistance <= 160)
                {
                    rightFront.setPower(-finalspeed);
                    rightRear.setPower(-finalspeed);
                    leftFront.setPower(finalspeed);
                    leftRear.setPower(finalspeed);
                }
                else
                {
                    telemetry.addData("Slow", rightFront.getPower());
                    telemetry.update();
                    rightFront.setPower(-finalspeed * 0.75);
                    rightRear.setPower(-finalspeed * 0.75);
                    leftFront.setPower(finalspeed * 0.75);
                    leftRear.setPower(finalspeed * 0.75);
                }
                telemetry.update();
            }
        }
        wrongAngle = getReorient(angle);
        telemetry.addData("Reorient", "%.2f", wrongAngle);
        telemetry.update();

        if (wrongAngle > 2 || wrongAngle < -2)
        {
            turn(0.30, angle);
        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        telemetry.addData("Finished Heading", curHeading);
        telemetry.update();
    }
}