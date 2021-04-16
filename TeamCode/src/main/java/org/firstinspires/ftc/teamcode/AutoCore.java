package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Correct SDK for second robot (GoBilda)
@TeleOp(name="AutoCore", group="Core")
@Disabled
public class AutoCore extends LinearOpMode {

    // All of our variables go here
    DcMotor rightRear = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor leftFront = null;
    DcMotor intake = null;
    DcMotorEx shooter1 = null;
    DcMotorEx shooter2 = null;
    DcMotor arm = null;
    Servo hand = null;
    Servo shoulder = null;
    Servo flipper = null;
    Servo camServo = null;

    //WebcamName webcam = null;

    // Voltage Sensor
    VoltageSensor batteryVoltSensor;

    // Our Inertial Measurement Unit (IMU)
    BNO055IMU ourIMU;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // Vuforia License key
    private static final String VUFORIA_KEY = "AVvnxxn/////AAABmRNOC9Fhg0J7mbnZziEjynVJQssG35HF0AMV42c143vA9h5ziXGhyZle+oQAmez4l2CBqZtikvsehIRV5uoERMn4PwO6KkGXCqc9iBrDuls9PSKsCu8yB4AnJzlVwHRZ/uvT8mw+MT9ylS+8S/pZBjVquqUtaa6kISwltzqP5fOIip1aTevvko2IVSuE8/EO8SQssF4SnB0xAMzgFz+3Zn8kNwp1l+tx/zultUJ9Je/al3NgeDn//IjWhw2D770PWwcSb39TGno0NWgy1YZhWmOQpuQsm1lC//H54DJFKTw/9UhLWbcloIsB/hQtWib/GIXOaLVfsJGTimgIdpgapXXJQv5CCmVaF5rCmhwY3p93";

    // Sets robot turn speed
    double autoTurnSpeed = 0;

    // Saved current orientation/heading
    double curHeading;
    // IMU Calibration File
    String imuCalibration = "RevIMUCalibration.json";


    // Encoder value for 1 inch movement
    double encoderDistance = (320 / ((98 * Math.PI) / 25.4));

    // Runtime
    public ElapsedTime runtime = new ElapsedTime();


    public AutoCore() {
        // Initialize base classes
    }

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Resets the runtime variable
        runtime.reset();

        // Hardware mapping goes here
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        flipper = hardwareMap.get(Servo.class, "flipper");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        camServo = hardwareMap.get(Servo.class, "camServo");

        batteryVoltSensor = hardwareMap.voltageSensor.get("Control Hub");

        // Brakes motors when not given power
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Starts the arms encoder... NEVER RESET
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Connect to the IMU in the primary hub
        ourIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Setup parameters which we will use our IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.calibrationDataFile = imuCalibration; // From calibration Sample
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Activate the parameters
        ourIMU.initialize(parameters);

        // Makes sure gyro is calibrated
        ourIMU.isGyroCalibrated();

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Was (10, 3, 0, 0)
        shooter1.setVelocityPIDFCoefficients(20,3,5,0);
        shooter2.setVelocityPIDFCoefficients(20,3,5,0); // TODO: Compare to DriverControl

        telemetry.addData("Ready To Start!", "");
        telemetry.update();
    }


    // 1 wheel spin is 320 encoder value for our robot
    // Original motor(60:1) is 1440
    // Wheels are on 40:3 gear ratio
    // 1440 = 54.5in


    // ***********************************************************************
    // ResetEncoder
    // ***********************************************************************
    // Starts motor encoders and resets their values
    // Used to save lines of code
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
    // StopEncoder
    // ***********************************************************************
    // Stops motor encoders
    // Used to save lines of code
    void stopEncoder() {
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                    rightFront.setPower(speed); // Was finalSpeed, but would get stuck in turn
                    rightRear.setPower(speed); // Was finalSpeed, but would get stuck in turn
                    leftFront.setPower(-speed); // Was finalSpeed, but would get stuck in turn
                    leftRear.setPower(-speed); // Was finalSpeed, but would get stuck in turn
                }
                else
                {
                    rightFront.setPower(speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    rightRear.setPower(speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    leftFront.setPower(-speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    leftRear.setPower(-speed); // Was finalSpeed * 0.75, but would get stuck in turn
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
                    rightFront.setPower(-speed); // Was finalSpeed, but would get stuck in turn
                    rightRear.setPower(-speed); // Was finalSpeed, but would get stuck in turn
                    leftFront.setPower(speed); // Was finalSpeed, but would get stuck in turn
                    leftRear.setPower(speed); // Was finalSpeed, but would get stuck in turn
                }
                else
                {
                    telemetry.addData("Slow", rightFront.getPower());
                    telemetry.update();
                    rightFront.setPower(-speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    rightRear.setPower(-speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    leftFront.setPower(speed); // Was finalSpeed * 0.75, but would get stuck in turn
                    leftRear.setPower(speed); // Was finalSpeed * 0.75, but would get stuck in turn
                }
                telemetry.update();
            }
        }
        wrongAngle = getReorient(angle);
        telemetry.addData("Reorient", "%.2f", wrongAngle);
        telemetry.update();

        if (wrongAngle > 3 || wrongAngle < -3) // Was 2 and -2
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


    // ***********************************************************************
    // TurnRight
    // ***********************************************************************
    // Causes robot to turn right
    // Used in autonomous
    void turnright(double speed, double angle) {

        angle += 360;

        checkOrientation();

        double finalspeed = 0;
        // Checks to see if the robot has gone full distance
        while (curHeading < angle) {
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
                finalspeed += 0.001;
            }
            checkOrientation();
            telemetry.addData("Heading Right", curHeading - 360);
            telemetry.update();

            // If the robot is less than 75% to the destination then it will go full speed
            if (curHeading < (angle)) {
                rightFront.setPower(-speed);
                rightRear.setPower(-speed);
                leftFront.setPower(speed);
                leftRear.setPower(speed);
            }
            // If the robot is past 75%, but less than 100% to the destination then it will slow to
            // half the target speed
/*
            else {
                rightFront.setPower(-speed * 0.75);
                rightRear.setPower(-speed * 0.75);
                leftFront.setPower(speed * 0.75);
                leftRear.setPower(speed * 0.75);
            }


 */


        }

        //angleCorrection(angle);

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }

    // ***********************************************************************
    // TurnLeft
    // ***********************************************************************
    // Causes robot to turn left
    // Used in autonomous
    void turnleft(double speed, double angle) {

        angle += 360;

        checkOrientation();

        double finalspeed = 0;
        // Checks to see if the robot has gone full distance
        while (curHeading > angle ) {
            if (finalspeed < speed)
            {
                finalspeed += 0.001;
                finalspeed += 0.001;
            }
            checkOrientation();
            telemetry.addData("Heading Left", curHeading - 360);
            telemetry.update();
            // If the robot is less than 75% to the destination then it will go full speed
            if (curHeading > angle) {
                rightFront.setPower(speed);
                rightRear.setPower(speed);
                leftFront.setPower(-speed);
                leftRear.setPower(-speed);
            }
            // If the robot is past 75%, but less than 100% to the destination then it will slow to
            // half the target speed
/*
            else if (curHeading > angle) {
                rightFront.setPower(speed * 0.75);
                rightRear.setPower(speed * 0.75);
                leftFront.setPower(-speed * 0.75);
                leftRear.setPower(-speed * 0.75);
            }


 */
        }

        angleCorrection(angle);

        // After robot has gone full distance, power is set to 0
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

    }

    // ***********************************************************************
    // NorthEast
    // ***********************************************************************
    // Causes robot to move diagonal to North East
    // Used in autonomous
    void ne(double speed, double distance) {

        // This is the inch to encoder distance
        distance *= encoderDistance;

        // Starts and Resets motor encoders
        resetEncoder();

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightRear.getCurrentPosition() < distance) {
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightRear.getCurrentPosition() < (distance * 0.9)) {
                rightRear.setPower(speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed * 0.5);
                leftRear.setPower(speed * 0.5);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightRear.getCurrentPosition() < (distance)) {
                rightRear.setPower(speed * 0.5);
                leftFront.setPower(speed * 0.5);
                rightFront.setPower(speed * 0.1);
                leftRear.setPower(speed * 0.1);
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
    // NorthWest
    // ***********************************************************************
    // Causes robot to move diagonal to North West
    // Used in autonomous
    void nw(double speed, double distance) {

        // This is the inch to encoder distance
        distance *= encoderDistance;

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
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() < (distance * 0.9)) {
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed * 0.5);
                leftFront.setPower(speed * 0.5);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() < (distance)) {
                rightFront.setPower(speed * 0.5);
                leftRear.setPower(speed * 0.5);
                rightRear.setPower(speed * 0.1);
                leftFront.setPower(speed * 0.1);
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
    // SouthWest
    // ***********************************************************************
    // Causes robot to move diagonal to South West
    // Used in autonomous
    void sw(double speed, double distance) {

        // This is the inch to encoder distance
        distance *= encoderDistance;

        // Starts and Resets motor encoders
        resetEncoder();

        // Gets the starting position of the motors and sets them to a variable
        // Can be used later for telemetry
        double rightFrontPos = rightFront.getCurrentPosition();
        double rightRearPos = rightRear.getCurrentPosition();
        double leftFrontPos = leftFront.getCurrentPosition();
        double leftRearPos = leftRear.getCurrentPosition();

        // Checks to see if the robot has gone full distance
        while (rightRear.getCurrentPosition() > -distance) {
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightRear.getCurrentPosition() > (-distance * 0.9)) {
                leftFront.setPower(-speed);
                rightRear.setPower(-speed);
                rightFront.setPower(-speed * 0.5);
                leftRear.setPower(-speed * 0.5);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightRear.getCurrentPosition() > (-distance)) {
                leftFront.setPower(-speed * 0.5);
                rightRear.setPower(-speed * 0.5);
                rightFront.setPower(-speed * 0.1);
                leftRear.setPower(-speed * 0.1);
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
    // SouthEast
    // ***********************************************************************
    // Causes robot to move diagonal to South East
    // Used in autonomous
    void se(double speed, double distance) {

        // This is the inch to encoder distance
        distance *= encoderDistance;

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
            // If the robot is less than 90% to the destination then it will go full speed
            if (rightFront.getCurrentPosition() > (-distance * 0.9)) {
                rightFront.setPower(-speed);
                leftRear.setPower(-speed);
                rightRear.setPower(-speed * 0.4);
                leftFront.setPower(-speed * 0.4);
            }
            // If the robot is past 90%, but less than 100% to the destination then it will slow to
            // half the target speed
            else if (rightFront.getCurrentPosition() > (-distance)) {
                rightFront.setPower(-speed * 0.5);
                leftRear.setPower(-speed * 0.5);
                rightRear.setPower(-speed * 0.1);
                leftFront.setPower(-speed * 0.1);
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
    // checkOrientation - gather the current orientation data
    // ***********************************************************************
    void checkOrientation() {
        // and the angles from that IMU
        Orientation angles;
        Acceleration gravAngles;

        // read the orientation of the robot
        angles = ourIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // and save the heading
        curHeading = -(angles.firstAngle);
    }
    // ******************************************* ****************************
    // AngleCorrection
    // ***********************************************************************
    // Corrects robots current angle
    // Used in autonomous
    void angleCorrection(double angle)
    {
        getReorient(angle);
        // Need to call getreorient and it needs to turn the robot at the end of each turn method
        // Need to figure out how to make turn methods work at 180 to -180
        telemetry.addData("Angle", curHeading - 360);
        checkOrientation();
        if (curHeading > angle + 1)
        {
            telemetry.addData("Correct Heading Left", curHeading - 360);
            turnleft(0.25, (angle - 360));
            sleep(50);
        }
        else if (curHeading < angle - 1)
        {
            telemetry.addData("Correct Heading Right", curHeading - 360);
            turnright(0.25,(angle - 360));
            sleep(50);
        }
    }

    // ***********************************************************************
    // getReorient
    // ***********************************************************************
    // how far wrong is current orientation
    public double getReorient(double targetAngle) {
        double baseOrient;

        // measure current heading against target as 0-360
        baseOrient = (curHeading - targetAngle + 720) % 360.0;

        // shift to -180 to +180
        if (baseOrient >= 180.0) {
            baseOrient -= 360.0;
        }
        else if (baseOrient <= -180.0)
        {
            baseOrient += 360.0;
        }

        return baseOrient;
    }

    // ***********************************************************************
    // ArmDown
    // ***********************************************************************
    // Moves wobble goal arm to down position to pick up wobble goal
    // Used in autonomous and TeleOp
    void armDown() {
        arm.setTargetPosition(1400);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.3);
        while (arm.getCurrentPosition() < 1400) {
            arm.setTargetPosition(1400);
        }
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("DownMode", arm.getMode());
        telemetry.addData("Target", arm.getTargetPosition());
        if (arm.getCurrentPosition() >= 1400) {
            arm.setPower(0);
        }
        telemetry.addData("Arm Position", arm.getCurrentPosition());

    }

    // ***********************************************************************
    // ArmUp
    // ***********************************************************************
    // Moves wobble goal arm to up position to transport wobble(or move arm out of the way)
    // Used in autonomous and TeleOp
    void armUp() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.3);
        while (arm.getCurrentPosition() > 0) {
            arm.setTargetPosition(0);
        }
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("UpMode", arm.getMode());
        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }
        telemetry.addData("Arm Position", arm.getCurrentPosition());
    }

    // ***********************************************************************
    // InitVuforia
    // ***********************************************************************
    // Initializes Vuforia
    // Used in autonomous
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // ***********************************************************************
    // InitAll
    // ***********************************************************************
    // Initializes all init methods for camera detection
    // Used in autonomous
    public void initAll() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

    }

    // ***********************************************************************
    // InitTfod
    // ***********************************************************************
    // Initializes Tfod
    // Used in autonomous
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // ***********************************************************************
    // ScanTarget
    // ***********************************************************************
    // Scans Target Ring Stack
    // Used in autonomous
    public String scanTarget()
    {
        String size = "";

            runtime.reset();
            telemetry.addData("Time", runtime.time());
            telemetry.update();

            while (runtime.time() < 1) {
                //atelemetry.addData("Time2", runtime.time());
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            size = recognition.getLabel();
                        }
                        telemetry.update();
                    }
                }
            }

        return size;
    }

    // ***********************************************************************
    // RingCount
    // ***********************************************************************
    // Moves based on ringCount
    // Used in autonomous
    public void ringCount(String size)
    {
        // If 1 ring is in the stack, this will run
        if (size == "Single")
        {
            telemetry.addData("size = ", size);
            telemetry.addData("Predicted Stack = ", 1);
            telemetry.update();

            moveback(0.25,24);
            turnleft(0.25, 0);
            moveback(0.25,24);
            armDown();
            handOpen();
            sleep(250);
            moveback(0.25,2);
            sleep(250);
            handClose();
            armUp();
            moveleft(0.25,3);
            turnright(0.25,0);
            moveforward(0.25, 18);

        }
        // If 4 rings are in the stack, this will run
        else if (size == "Quad")
        {
            telemetry.addData("size = ", size);
            telemetry.addData("Predicted Stack = ", 4);
            telemetry.update();
            moveback(0.25, 15);
            turnleft(0.25,0);
            moveback(0.25,10);
            turnleft(0.25, 0);
            moveback(0.25,20);
            turnright(0.37,85);
            moveforward(0.25,2);
            turnright(0.37,170);
            moveforward(0.25,4);
            armDown();
            handOpen();
            sleep(400);
            moveback(0.25, 4);
            handClose();
            armUp();
            sleep(250);
            checkOrientation();
            moveback(0.25, 12);
        }
        // If no rings are in the stack, this will run
        else
        {
            telemetry.addData("size = ", "None");
            telemetry.addData("Predicted Stack = ", 0);
            telemetry.update();
        }
    }

    // ***********************************************************************
    // HandOpen
    // ***********************************************************************
    // Opens Wobble Goal Hand
    // Used in autonomous and TeleOp
    public void handOpen()
    {
        hand.setPosition(0);
    }

    // ***********************************************************************
    // HandClose
    // ***********************************************************************
    // Closes Wobble Goal Hand
    // Used in autonomous and TeleOp
    public void handClose()
    {
        hand.setPosition(0.34);
    }

    // ***********************************************************************
    // FlipForward
    // ***********************************************************************
    // Moves Flipper to push ring torwards shooter
    // Used in autonomous and TeleOp
    void flipForward()
    {
        flipper.setPosition(0.13);
    }

    // ***********************************************************************
    // FlipBackward
    // ***********************************************************************
    // Moves Flipper back to reset position
    // Used in autonomous and TeleOp
    void flipBackward()
    {
        flipper.setPosition(0.02);
    }

    // ***********************************************************************
    // CamRight
    // ***********************************************************************
    // Rotates Camera to the right
    // Used in autonomous
    void camRight()
    {
        camServo.setPosition(0.25);
    }

    // ***********************************************************************
    // CamLeft
    // ***********************************************************************
    // Rotates Camera to the left
    // Used in autonomous
    void camLeft()
    {
        camServo.setPosition(0.65);
    }

    // ***********************************************************************
    // ShooterOn
    // ***********************************************************************
    // Turns Shooter on to hit Power Shots
    // Used in autonomous and TeleOp
    void shooterOn(double power)
    {
        double basepower = 702;
        telemetry.addData("Voltage", batteryVoltSensor.getVoltage());
        telemetry.update();
        /*
        if (batteryVoltSensor.getVoltage() > 13)
        {
            shooter1.setPower(0.43); // 42
            shooter2.setPower(0.43);
        }
        else if (batteryVoltSensor.getVoltage() > 12.5)
        {
            shooter1.setPower(0.45); // 43
            shooter2.setPower(0.45);
        }
        else if (batteryVoltSensor.getVoltage() > 12)
        {
            shooter1.setPower(0.47); //44
            shooter2.setPower(0.47);
        }
        else if (batteryVoltSensor.getVoltage() > 11.5)
        {
            shooter1.setPower(0.49);
            shooter2.setPower(0.49);
        }
        else if (batteryVoltSensor.getVoltage() > 11)
        {
            shooter1.setPower(0.51);
            shooter2.setPower(0.51);
        }
        else if (batteryVoltSensor.getVoltage() > 10.5)
        {
            shooter1.setPower(0.53);
            shooter2.setPower(0.53);
        }
        else if (batteryVoltSensor.getVoltage() > 10)
        {
            shooter1.setPower(0.55);
            shooter2.setPower(0.55);
        }
        else if (batteryVoltSensor.getVoltage() > 9.5)
        {
            shooter1.setPower(0.57);
            shooter2.setPower(0.57);
        }
        else if (batteryVoltSensor.getVoltage() > 9)
        {
            shooter1.setPower(0.59);
            shooter2.setPower(0.59);
        }
        else
        {
            shooter1.setPower(0.61);
            shooter2.setPower(0.61);
        }

         */
        shooter1.setVelocity(basepower + power);
        shooter2.setVelocity(basepower + power); // Was 920 before 1004 --> changed to 976
    }

    // ***********************************************************************
    // ShooterOff
    // ***********************************************************************
    // Turns Shooter Off
    // Used in autonomous and TeleOp
    void shooterOff()
    {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

}