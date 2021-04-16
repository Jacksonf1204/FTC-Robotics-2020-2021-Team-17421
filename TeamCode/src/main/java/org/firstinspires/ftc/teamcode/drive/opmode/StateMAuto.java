package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.opmode.StateMAuto.StateZone0.PICKUP_WOBBLE_PART_2;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(name = "StateMAuto", group = "Auto")
//@Disabled
public class StateMAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // Vuforia License key
    private static final String VUFORIA_KEY = "AVvnxxn/////AAABmRNOC9Fhg0J7mbnZziEjynVJQssG35HF0AMV42c143vA9h5ziXGhyZle+oQAmez4l2CBqZtikvsehIRV5uoERMn4PwO6KkGXCqc9iBrDuls9PSKsCu8yB4AnJzlVwHRZ/uvT8mw+MT9ylS+8S/pZBjVquqUtaa6kISwltzqP5fOIip1aTevvko2IVSuE8/EO8SQssF4SnB0xAMzgFz+3Zn8kNwp1l+tx/zultUJ9Je/al3NgeDn//IjWhw2D770PWwcSb39TGno0NWgy1YZhWmOQpuQsm1lC//H54DJFKTw/9UhLWbcloIsB/hQtWib/GIXOaLVfsJGTimgIdpgapXXJQv5CCmVaF5rCmhwY3p93";

    // IMU Calibration File
    String imuCalibration = "RevIMUCalibration.json";

    // Holds the size of the starter stack
    String size = "";

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime shotTime = new ElapsedTime();
    public ElapsedTime zone4wait = new ElapsedTime();
    public ElapsedTime moveWaitTime = new ElapsedTime();
    int number = 1;
    int shotsLeft = 3;
    int done = 0;
    int ringCount4Stack = 0;
    int waittime = 1;
    int enterShooting = 0;

    // Hardware Variables
    DcMotor intake = null;
    DcMotorEx shooter1 = null;
    DcMotorEx shooter2 = null;
    DcMotor arm = null;
    Servo hand = null;
    Servo flipper = null;

    // Blinkin LED Driver
    RevBlinkinLedDriver leds;

    // Enum defines our state (Possible steps)
    enum State {
        DRIVE_TO_LINE,
        DRIVE_TO_LINE2,
        MOVE_TO_STARTER_STACK,
        STACK_SIZE_0,
        STACK_SIZE_1,
        STACK_SIZE_4,
        PARK_ON_LINE,
        IDLE
    }

    enum StateShoot{
        FLIP_FORWARD,
        FLIP_BACKWARD,
        FLIP_FORWARD2,
        FLIP_BACKWARD2,
        IDLE
    }

    enum StateZone0{
        SHOOT_1,
        DEPOSIT_WOBBLE1,
        PICKUP_WOBBLE2,
        PICKUP_WOBBLE_PART_2,
        DEPOSIT_WOBBLE2,
        PARK_ON_LINE,
        PARK_ON_LINE_PART_2,
        IDLE
    }

    enum StateZone1{
        PICKUP_RING1,
        SHOOT1,
        DEPOSIT_WOBBLE1,
        DEPOSIT_WOBBLE1_PART_2,
        PICKUP_WOBBLE2,
        PICKUP_WOBBLE_PART_2,
        DEPOSIT_WOBBLE2,
        PARK_ON_LINE,
        IDLE
    }

    enum StateZone4{
        PICKUP_RING1,
        SHOOT1,
        PICKUP_RING2,
        SHOOT2,
        DEPOSIT_WOBBLE1,
        PICKUP_WOBBLE2,
        PICKUP_WOBBLE_PART_2,
        DEPOSIT_WOBBLE2,
        PARK_ON_LINE,
        IDLE
    }

    // Current State we are on
    // Default is IDLE
    State currentState = State.IDLE;
    StateShoot stateShoot = StateShoot.IDLE;
    StateZone0 stateZone0 = StateZone0.IDLE;
    StateZone1 stateZone1 = StateZone1.PICKUP_RING1;
    StateZone4 stateZone4 = StateZone4.IDLE;

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

    Pose2d startPose = new Pose2d(-64, -53, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initializes vuforia and tfod
        initAll();

        // Hardware Mapping
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        flipper = hardwareMap.get(Servo.class, "flipper");

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        // Motor directions and zero power behaviors
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        // Starts the arms encoder... NEVER RESET
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter motors PID
        shooter1.setVelocityPIDFCoefficients(225, 5, 70, 15); // Was 20, 3, 5, 0
        shooter2.setVelocityPIDFCoefficients(225, 5, 70, 15);

        drive.setPoseEstimate(startPose);

        FtcDashboard.getInstance().startCameraStream(tfod, 60);

        /*
         * Goal:
         * Start on outside line
         * Move to line to shoot high goals (Also scan rings while shooting)
         * Deposit wobble goal in zone
         * Pick up starter stack then shoot high goals (4 stack will pick up twice)
         * Pick up second wobble goal
         * Deposit second wobble goal in zone
         * Park on line
         */

        //----------------------------Trajectories--------------------------------------------------

        Trajectory driveToLine = drive.trajectoryBuilder(startPose, false)
                .lineTo(new Vector2d(-18,-53))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    // Turn on the shooter here
                    shooterOn(0);
                    handClose();
                })
                .build();

        Trajectory driveToLinePart2 = drive.trajectoryBuilder(driveToLine.end(),false)
                .lineTo(new Vector2d(0,-37))
                .build();

        Trajectory pickup1Ring = drive.trajectoryBuilder(driveToLinePart2.end(),false)
                .lineToLinearHeading(new Pose2d(-28,-37, Math.toRadians(180)))
                .addDisplacementMarker(0, () ->{
                    intakeOn();
                })
                .build();

        Trajectory pickup4Rings = drive.trajectoryBuilder(driveToLinePart2.end(),false)
                .splineToConstantHeading(new Vector2d( -28,-37), Math.toRadians(180),// x was -30 but might be going back to much extra
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH) // was 5
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0, ()->{
                    intakeOn();
                    shooterOn(-28);
                })
                .addDisplacementMarker(21,()->{
                    flipForward();
                })
                .addDisplacementMarker(23,()->{
                    flipBackward();
                })
                .build();

        Trajectory wobbleZone0 = drive.trajectoryBuilder(driveToLinePart2.end(), false)
                .lineToLinearHeading(new Pose2d(-5,-52, Math.toRadians(0)))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    shooterOff();
                    armDown();
                })
                .addDisplacementMarker(0.90,0, () ->{
                    // This will run when the program is 90% into the movement
                    handOpen();
                })
                .build();

        Trajectory wobbleZone1 = drive.trajectoryBuilder(pickup1Ring.end(), false)
                .lineToLinearHeading(new Pose2d(25,-28, Math.toRadians(0)))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    shooterOff();
                })
                .addDisplacementMarker(0.20,0, () ->{
                    // This will run when the program is 30% into the movement
                    armDown();
                })
                .addDisplacementMarker(0.95,0, () ->{
                    // This will run when the program is 90% into the movement
                    handOpen();
                })
                .build();

        Trajectory wobbleZone1Part2 = drive.trajectoryBuilder(wobbleZone1.end(),false)
                .lineTo(new Vector2d(14,-28))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    intakeOff();
                })
                .build();

        Trajectory wobbleZone4 = drive.trajectoryBuilder(pickup4Rings.end(), false)
                .lineToLinearHeading(new Pose2d(46,-52, Math.toRadians(1)))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    shooterOff();
                })
                .addDisplacementMarker(0.40, 0, () ->{
                    // This will run when the program is 20% into the movement
                    armDown();
                })
                .addDisplacementMarker(0.95,0, () ->{
                    // This will run when the program is 95% into the movement
                    handOpen();
                })
                .build();

        Trajectory grabSecondWobbleZone0 = drive.trajectoryBuilder(wobbleZone0.end(), true)
                .splineToLinearHeading(new Pose2d(-25,-30, Math.toRadians(181)),Math.toRadians(180))
                .addDisplacementMarker(0.0,0, () ->{
                    // The arm will drop down when the program is 30% through the path's distance
                    armDown();
                })
                .addTemporalMarker(0.5,0, () ->{
                    // The hand will open when the trajectory is 50% through the path
                    handOpen();
                })
                .build();

        Trajectory grabSecondWobbleZone0Part2 = drive.trajectoryBuilder(grabSecondWobbleZone0.end(), true)
                .lineToLinearHeading(new Pose2d(-36,-30, Math.toRadians(180)))
                .addDisplacementMarker(1.0,0, () ->{
                    // This will run when the program is 100% into the movement
                    handClose();
                })
                .addDisplacementMarker(1.0,1, () ->{
                    // This will run when the program is 0in into the movement
                    // TODO: Raise arm (Maybe halfway instead of completely) to transport
                })
                .build();

        Trajectory grabSecondWobbleZone1 = drive.trajectoryBuilder(wobbleZone1Part2.end(), true)
                .splineToLinearHeading(new Pose2d(-25,-30, Math.toRadians(181)), Math.toRadians(180))
                .build();

        Trajectory grabSecondWobbleZone1Part2 = drive.trajectoryBuilder(grabSecondWobbleZone1.end(), true)
                .lineToLinearHeading(new Pose2d(-36,-30, Math.toRadians(180)))
                .addDisplacementMarker(0.95,0, () ->{
                    // This will run when the program is 95% into the movement
                    handClose();
                })
                .build();

        Trajectory grabSecondWobbleZone4 = drive.trajectoryBuilder(wobbleZone4.end(), true)
                .splineToLinearHeading(new Pose2d(-25,-30, Math.toRadians(181)), Math.toRadians(180))
                .addDisplacementMarker(0,0, () ->{
                    // This will run when the program is 50% into the movement
                    armUp();
                })
                .addDisplacementMarker(0.25,0, () ->{
                    // This will run when the program is 25% into the movement
                    intakeOff();
                    armDown();
                })
                .build();

        Trajectory grabSecondWobbleZone4Part2 = drive.trajectoryBuilder(grabSecondWobbleZone4.end(), true)
                .lineToLinearHeading(new Pose2d(-34,-30, Math.toRadians(180)))
                .addDisplacementMarker(0.85,0, () ->{
                    // This will run when the program is 85% into the movement
                    handClose();
                })
                .build();

        Trajectory secondWobbleZone0 = drive.trajectoryBuilder(grabSecondWobbleZone0.end(), false)
                .lineToLinearHeading(new Pose2d(0,-60,Math.toRadians(0)))
                .addDisplacementMarker(0.95,0, () ->{
                    // This will run when the program is 90% into the movement
                    handOpen();
                })
                .build();

        Trajectory secondWobbleZone1 = drive.trajectoryBuilder(grabSecondWobbleZone1Part2.end(), false)
                .lineToLinearHeading(new Pose2d(22,-36, Math.toRadians(0)))
                .addDisplacementMarker(0.9,0, () ->{
                    // This will run when the program is 90% into the movement
                    handOpen();
                })
                .build();

        Trajectory secondWobbleZone4 = drive.trajectoryBuilder(grabSecondWobbleZone4Part2.end(), false)
                .lineToLinearHeading(new Pose2d(44,-54, Math.toRadians(0)))
                .addDisplacementMarker(0.05,0, () ->{
                    // This will run when the program is 5% into the movement
                    armUp();
                })
                .addDisplacementMarker(0.5,0, () ->{
                    // This will run when the program is 60% into the movement
                    armDown();
                })
                .addDisplacementMarker(0.9,0, () ->{
                    // This will run when the program is 90% into the movement
                    handOpen();
                })
                .build();

        Trajectory parkOnLine0 = drive.trajectoryBuilder(secondWobbleZone0.end(), true)
                .lineTo(new Vector2d(-15,-60))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                })
                .build();

        Trajectory parkOnLine0Part2 = drive.trajectoryBuilder(parkOnLine0.end(), true)
                .lineTo(new Vector2d(10,-30))
                .addDisplacementMarker(0, () ->{
                    // This will run when the program is 0in into the movement
                    handClose();
                    armUp();
                })
                .build();

        Trajectory parkOnLine1 = drive.trajectoryBuilder(secondWobbleZone1.end(), true)
                .lineTo(new Vector2d(10,-36))
                .addDisplacementMarker(0.8,0, () ->{
                    // This will run when the program is 80% into the movement
                    handClose();
                    armUp();
                })
                .build();

        Trajectory parkOnLine4 = drive.trajectoryBuilder(secondWobbleZone4.end(), true) // Was secondwobblezone4
                .lineTo(new Vector2d(10,-52))
                .addDisplacementMarker(0.8,0, () ->{
                    // This will run when the program is 15% into the movement
                    handClose();
                    armUp();
                })
                .build();

        //------------------------------------------------------------------------------------------
        waitForStart();
        if (isStopRequested()) return;

        currentState = State.DRIVE_TO_LINE;
        drive.followTrajectoryAsync(driveToLine);
        stateShoot = StateShoot.FLIP_FORWARD;
        StateZone0 stateZone0 = StateZone0.SHOOT_1;
        StateZone1 stateZone1 = StateZone1.PICKUP_RING1;
        StateZone4 stateZone4 = StateZone4.PICKUP_RING1;

        while(opModeIsActive() && !isStopRequested())
        {

            // Thoughts:
            // Switch inside switch? - For each starter stack size - Enters new switch after scanning
            // Within case and if statement(!drive.isBusy), add if statement to check which ring stack
            // 0,1,4. Change currentState to the case in which it will continue to the respective zone
            switch (currentState){
                case DRIVE_TO_LINE:
                    // Check if the drive class isn't busy
                    // After !isBusy() == true, we move to next state
                    // Make sure to use async follow function
                    if (!drive.isBusy()) {
                        handClose();
                        currentState = State.DRIVE_TO_LINE2;
                        drive.followTrajectoryAsync(driveToLinePart2);
                    }
                    break;

                case DRIVE_TO_LINE2:
                    if (!drive.isBusy())
                    {
                        if (number == 1)
                        {
                            shotTime.reset();
                            number = 2;
                        }
                        switch (stateShoot) {
                            case FLIP_FORWARD:
                                flipForward();
                                if (shotTime.time() > 0.35)
                                {
                                    stateShoot = StateShoot.FLIP_BACKWARD;
                                    shotTime.reset();
                                    shotsLeft--;
                                    size = scanTarget();
                                }
                                break;
                            case FLIP_BACKWARD:
                                flipBackward();
                                if (shotTime.time() > 0.35)
                                {
                                    if (shotsLeft > 0)
                                    {
                                        stateShoot = StateShoot.FLIP_FORWARD;
                                        shotTime.reset();
                                    }
                                    else
                                    {
                                        stateShoot = StateShoot.IDLE;
                                    }
                                }
                                break;
                            case IDLE:
                                number = 1;
                                shotsLeft = 3;
                                telemetry.addData("Size", size);
                                telemetry.update();

                                if (size == "Single")
                                {
                                    telemetry.addLine("1 Ring");
                                    telemetry.update();
                                    currentState = State.STACK_SIZE_1;
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                }
                                else if (size == "Quad")
                                {
                                    telemetry.addLine("4 Ring");
                                    telemetry.update();
                                    currentState = State.STACK_SIZE_4;
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                }
                                else
                                {
                                    telemetry.addLine("0 Ring");
                                    telemetry.update();
                                    currentState = State.STACK_SIZE_0;
                                    stateShoot = StateShoot.FLIP_FORWARD;
                                }
                                break;
                        }
                    }
                    break;

                case STACK_SIZE_0:
                    // Check if the drive class is busy following the trajectory
                    // Move on to next state when finished
                    switch (stateZone0){
                        case SHOOT_1:
                            if (!drive.isBusy()) {
                                if (number == 1) {
                                    shotTime.reset();
                                    number = 2;
                                }
                                switch (stateShoot) {
                                    case FLIP_FORWARD:
                                        flipForward();
                                        if (shotTime.time() > 0.35) {
                                            stateShoot = StateShoot.FLIP_BACKWARD;
                                            shotTime.reset();
                                        }
                                        break;
                                    case FLIP_BACKWARD:
                                        flipBackward();
                                        if (shotTime.time() > 0.35) {
                                            stateShoot = StateShoot.IDLE;
                                            shotTime.reset();
                                        }
                                        break;
                                    case IDLE:
                                        stateShoot = StateShoot.FLIP_FORWARD;
                                        stateZone0 = StateZone0.DEPOSIT_WOBBLE1;
                                        number = 1;
                                        break;
                                }
                            }
                            break;
                        case DEPOSIT_WOBBLE1:
                            if(!drive.isBusy()) {
                                stateZone0 = StateZone0.PICKUP_WOBBLE2;
                                drive.followTrajectoryAsync(wobbleZone0);
                            }
                            break;
                        case PICKUP_WOBBLE2:
                            if(!drive.isBusy()) {
                                stateZone0 = PICKUP_WOBBLE_PART_2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone0);
                            }
                            break;
                        case PICKUP_WOBBLE_PART_2:
                            if(!drive.isBusy()) {
                                stateZone0 = StateZone0.DEPOSIT_WOBBLE2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone0Part2);
                            }
                            break;
                        case DEPOSIT_WOBBLE2:
                            if(!drive.isBusy()) {
                                stateZone0 = StateZone0.PARK_ON_LINE;
                                drive.followTrajectoryAsync(secondWobbleZone0);
                            }
                            break;
                        case PARK_ON_LINE:
                            if (!drive.isBusy()){
                                stateZone0 = StateZone0.PARK_ON_LINE_PART_2;
                                drive.followTrajectoryAsync(parkOnLine0);
                            }
                            break;
                        case PARK_ON_LINE_PART_2:
                            if (!drive.isBusy()){
                                stateZone0 = StateZone0.IDLE;
                                drive.followTrajectoryAsync(parkOnLine0Part2);
                                telemetry.addLine("0 Stop");
                                telemetry.update();
                            }
                            break;
                        case IDLE:
                            if (!drive.isBusy() && !arm.isBusy()){
                                currentState = State.IDLE;
                            }
                            break;
                        }
                    break;
                case STACK_SIZE_1:
                    switch (stateZone1) {
                        case PICKUP_RING1:
                            if (!drive.isBusy()) {
                                shooterOn(-28);
                                stateZone1 = StateZone1.SHOOT1;
                                drive.followTrajectoryAsync(pickup1Ring);
                            }
                            break;
                        case SHOOT1:
                            if (!drive.isBusy()) {
                                if (number == 1) {
                                    shotTime.reset();
                                    number = 2;
                                }
                                switch (stateShoot) {
                                    case FLIP_FORWARD:
                                        if (shotTime.time() > 2) {
                                            flipForward();
                                            if (shotTime.time() > 2.35) {
                                                stateShoot = StateShoot.FLIP_BACKWARD;
                                                shotTime.reset();
                                            }
                                        }
                                        break;
                                    case FLIP_BACKWARD:
                                        flipBackward();
                                        if (shotTime.time() > 0.35) {
                                            stateShoot = StateShoot.FLIP_FORWARD2;
                                            shotTime.reset();
                                        }
                                        break;
                                    case FLIP_FORWARD2:
                                        flipForward();
                                        if (shotTime.time() > 0.35) {
                                            stateShoot = StateShoot.FLIP_BACKWARD2;
                                            shotTime.reset();
                                        }
                                        break;
                                    case FLIP_BACKWARD2:
                                        flipBackward();
                                        if (shotTime.time() > 0.35) {
                                            stateShoot = StateShoot.IDLE;
                                        }
                                    case IDLE:
                                        stateShoot = StateShoot.FLIP_FORWARD;
                                        stateZone1 = StateZone1.DEPOSIT_WOBBLE1;
                                        number = 1;
                                        break;
                                }
                            }
                            break;
                        case DEPOSIT_WOBBLE1:
                            if (!drive.isBusy()) {
                                stateZone1 = StateZone1.DEPOSIT_WOBBLE1_PART_2;
                                drive.followTrajectoryAsync(wobbleZone1);
                            }
                            break;
                        case DEPOSIT_WOBBLE1_PART_2:
                            if (!drive.isBusy()) {
                                stateZone1 = StateZone1.PICKUP_WOBBLE2;
                                drive.followTrajectoryAsync(wobbleZone1Part2);
                            }
                            break;
                        case PICKUP_WOBBLE2:
                            if (!drive.isBusy() && !arm.isBusy()) {
                                stateZone1 = StateZone1.PICKUP_WOBBLE_PART_2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone1);
                            }
                            break;
                        case PICKUP_WOBBLE_PART_2:
                            if (!drive.isBusy()) {
                                stateZone1 = StateZone1.DEPOSIT_WOBBLE2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone1Part2);
                            }
                            break;
                        case DEPOSIT_WOBBLE2:
                            if (!drive.isBusy()) {
                                stateZone1 = StateZone1.PARK_ON_LINE;
                                drive.followTrajectoryAsync(secondWobbleZone1);
                            }
                            break;
                        case PARK_ON_LINE:
                            if (!drive.isBusy()) {
                                stateZone1 = StateZone1.IDLE;
                                drive.followTrajectoryAsync(parkOnLine1);
                                telemetry.addLine("1 Stop");
                                telemetry.update();
                            }
                            break;
                        case IDLE:
                            if (!drive.isBusy() && !arm.isBusy()) {
                                currentState = State.IDLE;
                            }
                            break;
                    }
                    break;
                case STACK_SIZE_4:
                    switch (stateZone4){
                        case PICKUP_RING1:
                            if (!drive.isBusy()) {
                                drive.followTrajectoryAsync(pickup4Rings);
                                stateZone4 = StateZone4.SHOOT1;
                            }
                            break;
                        case SHOOT1:
                            if (!drive.isBusy()) {
                                if (number == 1) {
                                    shotTime.reset();
                                    number = 2;
                                    enterShooting = 1;
                                }
                                if (enterShooting == 1) {
                                    switch (stateShoot) {
                                        case FLIP_FORWARD:
                                            flipForward();
                                            if (shotTime.time() > 0.35) {
                                                stateShoot = StateShoot.FLIP_BACKWARD;
                                                shotTime.reset();
                                                shotsLeft--;
                                            }
                                            break;
                                        case FLIP_BACKWARD:
                                            flipBackward();
                                            if (shotTime.time() > 0.35) {
                                                if (shotsLeft > 0) {
                                                    stateShoot = StateShoot.FLIP_FORWARD;
                                                    shotTime.reset();
                                                } else {
                                                    stateShoot = StateShoot.IDLE;
                                                }
                                            }
                                            break;
                                        case IDLE:
                                            number = 1;
                                            stateZone4 = StateZone4.DEPOSIT_WOBBLE1;
                                            stateShoot = StateShoot.FLIP_FORWARD;
                                            moveWaitTime.reset();
                                            break;
                                    }
                                }
                            }
                            break;
                        case DEPOSIT_WOBBLE1:
                            if (!drive.isBusy()) {
                                stateZone4 = StateZone4.PICKUP_WOBBLE2;
                                drive.followTrajectoryAsync(wobbleZone4);
                            }
                            break;
                        case PICKUP_WOBBLE2:
                            if (!drive.isBusy()) {
                                stateZone4 = StateZone4.PICKUP_WOBBLE_PART_2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone4);
                            }
                            break;
                        case PICKUP_WOBBLE_PART_2:
                            if (!drive.isBusy()) {
                                stateZone4 = StateZone4.DEPOSIT_WOBBLE2;
                                drive.followTrajectoryAsync(grabSecondWobbleZone4Part2);
                            }
                            break;
                        case DEPOSIT_WOBBLE2:
                            if (!drive.isBusy()) {
                                if (number == 1)
                                {
                                    zone4wait.reset();
                                    number = 0;
                                }
                                if (zone4wait.time() > 0.5) {
                                    stateZone4 = StateZone4.PARK_ON_LINE;
                                    drive.followTrajectoryAsync(secondWobbleZone4);
                                    number = 1;
                                }
                            }
                            break;
                        case PARK_ON_LINE:
                            if (!drive.isBusy()) {
                                stateZone4 = StateZone4.IDLE;
                                drive.followTrajectoryAsync(parkOnLine4);
                            }
                            break;
                        case IDLE:
                            if (!drive.isBusy() && !arm.isBusy()) {
                                currentState = State.IDLE;
                            }
                            break;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // Current state does not change once in IDLE
                    // End of Autonomous
                    requestOpModeStop();
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // Update drive continuously in the background regardless of state
            drive.update();

        }

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
        String size = new String();

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


        return size;
    }


    // ***********************************************************************
    // ArmDown
    // ***********************************************************************
    // Moves wobble goal arm to down position to pick up wobble goal
    // Used in autonomous and TeleOp
    void armDown() {
        arm.setTargetPosition(2000);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        arm.setTargetPosition(2000);

        if (arm.getCurrentPosition() >= 2000) {
            arm.setPower(0);
        }
    }

    // ***********************************************************************
    // ArmUp
    // ***********************************************************************
    // Moves wobble goal arm to up position to transport wobble(or move arm out of the way)
    // Used in autonomous and TeleOp
    void armUp() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        arm.setTargetPosition(0);

        if (arm.getCurrentPosition() <= 0) {
            arm.setPower(0);
        }

    }

    // ***********************************************************************
    // ArmHalf
    // ***********************************************************************
    // Moves wobble goal arm to halfway position to transport wobble(or move arm out of the way)
    // Used in autonomous and TeleOp
    void armHalf() {
        arm.setTargetPosition(450);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);
        arm.setTargetPosition(450);

        if (arm.getCurrentPosition() <= 450) {
            arm.setPower(0);
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
        hand.setPosition(0.4);
    }

    // ***********************************************************************
    // FlipForward
    // ***********************************************************************
    // Moves Flipper to push ring torwards shooter
    // Used in autonomous and TeleOp
    void flipForward()
    {
        flipper.setPosition(0.13);
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("RED"));
    }

    // ***********************************************************************
    // FlipBackward
    // ***********************************************************************
    // Moves Flipper back to reset position
    // Used in autonomous and TeleOp
    void flipBackward()
    {
        flipper.setPosition(0.00);
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf("GOLD"));
    }

    // ***********************************************************************
    // IntakeOn
    // ***********************************************************************
    // Turns the intake on to collect rings
    // Used in autonomous
    void intakeOn() {
        intake.setPower(-1);
    }

    // ***********************************************************************
    // IntakeOff
    // ***********************************************************************
    // Turns the intake off to collect rings
    // Used in autonomous
    void intakeOff() {
        intake.setPower(0);
    }

    void shooterOn(int adjPower)
    {
        double basepower = 744;

        shooter1.setVelocity(basepower + adjPower);
        shooter2.setVelocity(basepower + adjPower);
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
    public int shoot3Rings()
    {
        if (number == 1)
        {
            shotTime.reset();
        }
        switch (stateShoot) {
            case FLIP_FORWARD:
                flipForward();
                if (shotTime.time() > 0.5)
                {
                    stateShoot = StateShoot.FLIP_BACKWARD;
                    shotTime.reset();
                    shotsLeft--;
                }
                break;
            case FLIP_BACKWARD:
                flipBackward();
                if (shotTime.time() > 0.5)
                {
                    if (shotsLeft > 0)
                    {
                        stateShoot = StateShoot.FLIP_FORWARD;
                        shotTime.reset();
                    }
                    else
                    {
                        stateShoot = StateShoot.IDLE;
                    }
                }
                break;
            case IDLE:
                done = 1;
                break;
        }
        return done;
    }
}
