package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="RR RobotCore", group="Core")
@Disabled
public class RRRobotCore extends OpMode {

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
    Servo flipper = null;

    // Blinkin LED Driver
    RevBlinkinLedDriver leds;

    // Our Inertial Measurement Unit (IMU)
    BNO055IMU ourIMU;

    // Saved current orientation/heading
    double curHeading;
    // IMU Calibration File
    String imuCalibration = "RevIMUCalibration.json";

    VoltageSensor batteryVoltSensor;

    // Encoder value for 1 inch movement
    double encoderDistance = (320 / ((98 * Math.PI) / 25.4));

    // Joystick threshold to start adding power
    double threshold = 0.05;

    public RRRobotCore() {
        // Initialize base classes
    }

    @Override
    public void init() {

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
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

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
        arm.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter1.setVelocityPIDFCoefficients(225, 5, 70, 15); // 15 3 5 0 top with 300 p
        shooter2.setVelocityPIDFCoefficients(225, 5, 70, 15); // 15 3 5 0 top with 300 p TODO: Compare to AutoCore
        // Connect to the IMU in the primary hub
        ourIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Setup parameters which we will use our IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.calibrationDataFile = imuCalibration; // From calibration Sample
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        batteryVoltSensor = hardwareMap.voltageSensor.get("Control Hub");

        // Activate the parameters
        ourIMU.initialize(parameters);


    }

    @Override
    public void start() {

        // Actions that are common to both TeleOp and Autonomous Modes go here


    }

    @Override
    public void loop() {

        // Actions that are common to both TeleOp and Autonomous Modes go here


    }

    @Override
    public void stop() {


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
    // FlipForward
    // ***********************************************************************
    // Moves Flipper to push ring torwards shooter
    // Used in autonomous and TeleOp
    void flipForward() {
        flipper.setPosition(0.13);
    }

    // ***********************************************************************
    // FlipBackward
    // ***********************************************************************
    // Moves Flipper back to reset position
    // Used in autonomous and TeleOp
    void flipBackward() {
        flipper.setPosition(0.03);
    }


    // ***********************************************************************
    // HandClose
    // ***********************************************************************
    // Moves hand to close position
    // Used in autonomous and TeleOp
    void handClose() {
        hand.setPosition(0.4);
    }

    // ***********************************************************************
    // HandOpen
    // ***********************************************************************
    // Moves hand to open position
    // Used in autonomous and TeleOp
    void handOpen() {
        hand.setPosition(0);
    }

    // ***********************************************************************
    // ShooterOn
    // ***********************************************************************
    // Turns Shooter on to hit Power Shots
    // Used in autonomous and TeleOp
    void shooterOn(int power) {
        double basepower = 744;

        shooter1.setVelocity(basepower + power);
        shooter2.setVelocity(basepower + power);
    }

    // ***********************************************************************
    // ShooterOff
    // ***********************************************************************
    // Turns Shooter Off
    // Used in autonomous and TeleOp
    void shooterOff() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    void ledHeartbeat(){
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1625));
    }

    void ledScanner(){
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1595));
    }
}