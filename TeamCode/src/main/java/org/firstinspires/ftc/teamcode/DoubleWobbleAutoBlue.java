package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.sql.Driver;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//A: (1,4 on dice) Close - 0 rings
//B: (2,5 on dice) Middle - 1 rings
//C: (3,6 on dice) Far - 4 rings

@Autonomous(name = "DoubleWobbleAutoBlue")
public class DoubleWobbleAutoBlue extends OpMode{

    private int path;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ARFWHLf/////AAABmQYBi+Yt9UfXhQeC8EeUpylMd+Pha6aQHw+i5Pyw28Fs7CaBACsFzXFNGv7p2jfwf9sn5zAO1CNrRa6XilwVANqg6g+mkwciKF38WPZGG6j88PDkkTkH6Sq6RM/VeeYCf+BikiEWGjM/BS5u8FlfvYERSQ9En9Hn8ootiBHXeNZrGl4BZwIfpt0LachcG2DAadZSZbtGV6evUlpC++Sx6JvuERscDOFVo1YdV2MHovW82LVRgp8Xctfsdr5euTXVkCT7d2C9I1X7D+y4mjjZSd4N6VMkuQYAXTsIU+RSW+OeSXtRRkFdZ7O5fCM6bUNgGUdyT7vSRUWh3A3qVTGLNT+exhPciVkD9yW/xW5yMvMk";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    BNO055IMU imu;
    Orientation angles;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "FrontRight";
    String horizontalEncoderName = "BackRight";

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;

    private double StartingXPosition;
    private double StartingYPosition;
    private double StartingRotation;

    final double COUNTS_PER_INCH = 307.699557;

    private double lastDistanceToTarget = 0;
    private double distanceToTarget;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;

    private DcMotor IntakeMotor;
    private DcMotor ShooterMotor;
    private DcMotor TransferMotor;

    private DcMotor WobbleMotor;

    private Servo ShooterFeedingServo;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private VoltageSensor batteryVoltageSensor;
    private double batteryVoltage;
    private double DEFAULT_MOVEMENT_SPEED = 1.0;
    private double DEFAULT_TURN_SPEED = 1.0;
    private static final double BATTERY_VOLTAGE_COMP_X1 = 13.0;
    private static final double BATTERY_VOLTAGE_COMP_X2 = 14.0;
    private static final double BATTERY_VOLTAGE_COMP_Y1 = 1.0;
    private static final double BATTERY_VOLTAGE_COMP_Y2 = 0.8;

    private double startTime;
    private double currentTime;
    private long lastUpdateTime = 0;

    private int autoState = INIT_STATE;
    private int lastAutoState = NO_STATE;
    private static final int NO_STATE = -1;
    private static final int INIT_STATE = 0;
    private static final int STARTING_DRIVE = 1;
    private static final int STARTING_TURN = 2;
    private static final int SECOND_DRIVE = 3;
    private static final int SECOND_TURN = 4;
    private static final int DRIVE_BACKWARDS = 5;
    private static final int DRIVE_TO_STRAFE = 6;
    private static final int SHOOT_RINGS = 8;
    private static final int DRIVE_TO_SHOOT = 9;
    private static final int DRIVE_TO_TURN = 10;
    private static final int ROTATE_ROBOT = 20;
    private static final int RELEASE_WOBBLE = 25;
    private static final int DRIVE_AWAY_FROM_WOBBLE = 27;
    private static final int DRIVE_TO_LOW_GOAL = 30;
    private static final int DUMP_RINGS = 40;
    private static final int DRIVE_FROM_GOAL = 43;
    private static final int RAISE_RINGS = 46;
    private static final int DRIVE_TO_LINE = 50;
    private static final int PARK_ON_LINE = 60;
    private static final int DRIVE_TO_START_WALL = 70;
    private static final int DRIVE_TO_SECOND_WOBBLE = 80;
    private static final int DRIVE_FORWARD_RIGHT = 90;
    private static final int DELIVER_SECOND_WOBBLE = 100;
    private static final int SECOND_DRIVE_BACKWARDS = 110;

    private static final double DECELERATION_START_POINT = 24;
    private static final double DECELERATION_ZERO_POINT = -1;
    private static final double TURNING_DECELERATION_START_POINT = 100;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1;

    //SHOOTER VARIABLES ----------------------------------------------------------------------------

    private boolean firstPressShooterToggleButton = true;
    private boolean firstPressShooterFeedingToggleButton = true;
    private boolean firstPressShooterSpeedToggleButton = true;
    private boolean shooterOn = false;
    private boolean shooterIsFast = true;
    private boolean shooterFeedingOn = false;
    private boolean firstRunShootThreeRings = true;

    private double launchPosition = .53;
    private double storePosition = 1;
    private double downPosition = .58;

    double startShootingTime = 0;
    double globalStartTime;

    private double shooterTicksPerRevolution = 103.6;

    //Wobble Variables -----------------------------------------------------------------------------
    private boolean firstPressWobbleButton = true;
    private boolean wobbleUp = true;
    private boolean slowDriveSpeed = false;
    private int globalWobbleOffset = 0;
    private boolean wobbleHasMoved = false;


    @Override
    public void init() {
        batteryVoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");

        initVuforia();
        initTfod();

        initializeDriveTrain();
        initializeOdometry();
        initializeIMU();

        initializeShooter();

        initializeWobble();

        telemetry.update();
    }

    public void init_loop() {
        telemetry.addData("Version Number", "2/06/21");
        choosePath();
    }

    public void start() {
        checkBatteryVoltage();

        //startWobble();

        startIMU();
        startOdometry();

        startShooter();

        globalStartTime = getRuntime();

        //ShooterMotor.setPower(.8);
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }
    public void loop() {
        currentTime = getRuntime();

        checkOdometry();
        goToPositionByTime(-15, 45, 1, .3, 90, 1, INIT_STATE, STARTING_DRIVE);

        if (path == 0){
            goToPositionByTime(-20, 66, .4, .3, 90, 1.75, STARTING_DRIVE, DRIVE_BACKWARDS);
            goToPositionByTime(-20, 57, .5, .3, 90, 1, DRIVE_BACKWARDS, DRIVE_TO_SHOOT);
            goToPositionByTime(23, 56, .75, .3, 90, 3, DRIVE_TO_SHOOT, SHOOT_RINGS);
            shootThreeRings(SHOOT_RINGS, DRIVE_TO_START_WALL,9);
            goToPositionByTime(32, 0, .5, .3, 90, 2, DRIVE_TO_START_WALL, DRIVE_TO_SECOND_WOBBLE);
            goToPositionByTime(15, -1, .5, .3, 120, 1.5, DRIVE_TO_SECOND_WOBBLE, DELIVER_SECOND_WOBBLE);
            goToPositionByTime(-18, 60, .4, .3, 120, 3, DELIVER_SECOND_WOBBLE, SECOND_DRIVE_BACKWARDS);
            goToPositionByTime(-18, 53, .5, .3, 90, 1, SECOND_DRIVE_BACKWARDS, DRIVE_TO_LINE);
            goToPositionByTime(20, 70, .75, .3, 90, 3, DRIVE_TO_LINE, DRIVE_TO_LINE);
        }
        else if (path == 1) {
            goToPositionByTime(-1, 90, .5, .3, 90, 3, STARTING_DRIVE, DRIVE_BACKWARDS);
            goToPositionByTime(-1, 81, .5, .3, 90, 1, DRIVE_BACKWARDS, DRIVE_TO_SHOOT);
            goToPositionByTime(25, 55, .75, .3, 90, 4, DRIVE_TO_SHOOT, SHOOT_RINGS);
            shootThreeRings(SHOOT_RINGS, DRIVE_TO_START_WALL,9);
            goToPositionByTime(32, 0, .5, .3, 90, 2, DRIVE_TO_START_WALL, DRIVE_TO_SECOND_WOBBLE);
            goToPositionByTime(9, -1, .5, .3, 90, 1.5, DRIVE_TO_SECOND_WOBBLE, DRIVE_FORWARD_RIGHT);
            goToPositionByTime(18, 48, .5, .3, 90, 2, DRIVE_FORWARD_RIGHT, DELIVER_SECOND_WOBBLE);
            goToPositionByTime(0, 85, .5, .3, 90, 3, DELIVER_SECOND_WOBBLE, DRIVE_TO_LINE);
            goToPositionByTime(0, 72, .75, .3, 90, 3, DRIVE_TO_LINE, DRIVE_TO_LINE);
        }
        else if (path == 4) {
            goToPositionByTime(-24, 112, .5, .3, 90, 2.5, STARTING_DRIVE, DRIVE_BACKWARDS);
            goToPositionByTime(-23, 100, .5, .3, 90, .4, DRIVE_BACKWARDS, DRIVE_TO_SHOOT);
            goToPositionByTime(24, 54, .7, .3, 90, 3, DRIVE_TO_SHOOT, SHOOT_RINGS);
            shootThreeRings(SHOOT_RINGS, DRIVE_TO_START_WALL,9);
            goToPositionByTime(30, -1, .5, .3, 90, 2, DRIVE_TO_START_WALL, DRIVE_TO_SECOND_WOBBLE);
            goToPositionByTime(8, -3, .5, .3, 90, 1.5, DRIVE_TO_SECOND_WOBBLE, DRIVE_FORWARD_RIGHT);
            goToPositionByTime(20, 48, .5, .3, 90, 1.5, DRIVE_FORWARD_RIGHT, DELIVER_SECOND_WOBBLE);
            goToPositionByTime(-23, 106, .5, .3, 120, 3.8, DELIVER_SECOND_WOBBLE, DRIVE_TO_LINE);
            goToPositionByTime(-23, 70, .5, .3, 90, 3, DRIVE_TO_LINE, DRIVE_TO_LINE);
        }

        if (currentTime - globalStartTime > 29) {
            TransferMotor.setTargetPosition(0);
        }

        telemetry.addData("Path", path);
        telemetry.addData("Current State", autoState);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (globalPositionUpdate != null) {
            globalPositionUpdate.stop();
        }
        stopTfod();
    }

    //Battery --------------------------------------------------------------------------------------

    private void checkBatteryVoltage() {
        batteryVoltage = batteryVoltageSensor.getVoltage();

        // Max motor power is decreased if battery voltage is greater than a certain threshold,
        // and is linearly decreased at a larger amount as voltage increases.
        if(batteryVoltage > BATTERY_VOLTAGE_COMP_X1) {
            DEFAULT_MOVEMENT_SPEED = ((BATTERY_VOLTAGE_COMP_Y2 - BATTERY_VOLTAGE_COMP_Y1)
                    / (BATTERY_VOLTAGE_COMP_X2 - BATTERY_VOLTAGE_COMP_X1))
                    * (batteryVoltage - BATTERY_VOLTAGE_COMP_X1) + BATTERY_VOLTAGE_COMP_Y1;
            DEFAULT_TURN_SPEED = DEFAULT_MOVEMENT_SPEED;
        } else {
            DEFAULT_MOVEMENT_SPEED = 1.0;
            DEFAULT_TURN_SPEED = 1.0;
        }
    }

    //Vuforia --------------------------------------------------------------------------------------

    private void choosePath() {
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

                    if (recognition.getLabel().equals("Quad")) {
                        path = 4;
                    }
                    else if (recognition.getLabel().equals("Single")) {
                        path = 1;
                    }
                    else {
                        path = 0;
                    }

                }
                if (updatedRecognitions.size() == 0) {
                    path = 0;
                }
            }
        }
        else {
            path = 0;
        }

        telemetry.addData("Path", path);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
    }

    private void stopTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //Drivetrain -----------------------------------------------------------------------------------

    private void initializeDriveTrain() {
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void applyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        double fl_power_raw = -movement_y+movement_turn+movement_x;
        double bl_power_raw = -movement_y+movement_turn-movement_x;
        double br_power_raw = movement_y+movement_turn-movement_x;
        double fr_power_raw = movement_y+movement_turn+movement_x;

        //find the maximum of the powers
        double maxRawPower = Math.abs(fl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(fr_power_raw) > maxRawPower){ maxRawPower = Math.abs(fr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        fl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        fr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        FrontLeft.setPower(-fl_power_raw);
        BackLeft.setPower(-bl_power_raw);
        BackRight.setPower(-br_power_raw);
        FrontRight.setPower(-fr_power_raw);
    }

    //ODOMETRY -------------------------------------------------------------------------------------

    private void initializeOdometry() {
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //These values also affect the drive motors so we also reversed FrontRight
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //StartingXPosition = Double.parseDouble(ReadWriteFile.readFile(startingXpositionFile).trim());
        //StartingYPosition = Double.parseDouble(ReadWriteFile.readFile(startingYpositionFile).trim());
        //StartingRotation = Double.parseDouble(ReadWriteFile.readFile(startingθpositionFile).trim());
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 90;

    }

    private void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 25);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    private void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = -(globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        if (RobotRotation < 0){
            RobotRotation += 360;
        }

        double robotXpositionRound = (Math.round (100*RobotXPosition));
        double robotYpositionRound = (Math.round (100*RobotYPosition));
        double robotRotationRound = (Math.round (100*RobotRotation));

        telemetry.addData("X", (robotXpositionRound / 100));
        telemetry.addData("Y", (robotYpositionRound / 100));
        telemetry.addData("θ", (robotRotationRound / 100));

        telemetry.addData("Vertical Left Encoder", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Encoder", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontal.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
    }

    //GoToPosition ---------------------------------------------------------------------------------

    private void goToPosition(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);

//        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget - Math.toRadians(RobotRotation)
//                + Math.toRadians(90));
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - Math.toRadians(RobotRotation));

        double relativeXToPoint = 2 * Math.sin(relativeAngleToPoint);
        double relativeYToPoint = Math.cos(relativeAngleToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double yDecelLimiter = Range.clip(Math.abs((distanceToTarget - DECELERATION_ZERO_POINT)
                / (DECELERATION_START_POINT - DECELERATION_ZERO_POINT)), 0, 1);
        double xDecelLimiter = Range.clip(yDecelLimiter * X_SPEED_MULTIPLIER, 0, 1);

        double relativeTurnAngle = AngleWrap(Math.toRadians(preferredAngle)-Math.toRadians(RobotRotation));
        double turnDecelLimiter = Range.clip((Math.abs(Math.toDegrees(relativeTurnAngle)) - TURNING_DECELERATION_ZERO_POINT)
                / (TURNING_DECELERATION_START_POINT - TURNING_DECELERATION_ZERO_POINT), 0, 1);

        movement_x = movementXPower * Range.clip(maxMovementSpeed, -xDecelLimiter, xDecelLimiter);
        movement_y = movementYPower * Range.clip(maxMovementSpeed, -yDecelLimiter, yDecelLimiter);

        if (distanceToTarget < 1 && Math.abs(relativeAngleToPoint) < 5) {
            movement_turn = 0;
        }
        else {
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(TURNING_DECELERATION_START_POINT), -1, 1) * maxTurnSpeed;
        }
        telemetry.addData("relativeTurnAngle", relativeTurnAngle);
        telemetry.addData("turnDecelLimiter", turnDecelLimiter);
        telemetry.addData("relativeXToPoint", relativeXToPoint);
        telemetry.addData("relativeYToPoint", relativeYToPoint);
        telemetry.addData("X Movement", movement_x);
        telemetry.addData("Y Movement", movement_y);
        telemetry.addData("Turn Movement", movement_turn);

        lastDistanceToTarget = distanceToTarget;

        applyMovement();
    }

    private void goToPositionByTime(double x, double y, double preferredAngle, double timeout, int thisState, int nextState) {
        double maxMovementSpeed = 1.0;
        double maxTurnSpeed = 1.0;

        goToPositionByTime(x, y, maxMovementSpeed, maxTurnSpeed, preferredAngle, timeout, thisState, nextState);
    }

    private void goToPositionByTime(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle, double timeout, int thisState, int nextState) {
        // exit method if auto is not in this state
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        // setup timer, set startTime variable


        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
        goToPosition(x, y, maxMovementSpeed, maxTurnSpeed, preferredAngle);
    }

    private static double AngleWrap(double angle){

        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }

        return angle;
    }

    //IMU ------------------------------------------------------------------------------------------

    private void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();
    }

    private void startIMU() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        if(angles != null) {

            telemetry.addLine()
                    .addData("heading", new Func<Object>() {
                        @Override
                        public Object value() {
                            double normalizedAngle = angles.firstAngle;

                            if (normalizedAngle < 0) {
                                normalizedAngle += 360;
                            }
                            return normalizedAngle; // formatAngle(angles.angleUnit, );
                        }
                    })
                /*.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle))*/;
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //SHOOTER --------------------------------------------------------------------------------------

    private void initializeShooter() {
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterFeedingServo = hardwareMap.servo.get("ShooterFeedingServo");
        ShooterFeedingServo.setPosition(downPosition);

        TransferMotor = hardwareMap.dcMotor.get("TransferMotor");
        TransferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TransferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TransferMotor.setTargetPosition(0);
        TransferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TransferMotor.setPower(.4);
    }
    private void startShooter() {
        ShooterMotor.setPower(.75);
        ShooterFeedingServo.setPosition(storePosition);
        TransferMotor.setTargetPosition(-185);
    }

    /*private void shootOneRing(int thisState, int nextState, double timeout) {
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        ShooterFeedingMotor.setPower(-1);

        if (currentTime - timeout > startTime) {
            ShooterFeedingMotor.setPower(0);
            autoState = nextState;
        }
    }*/

    private void shootThreeRings(int thisState, int nextState, double timeout) {

        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        if (firstRunShootThreeRings) {
            startShootingTime = getRuntime();
            currentTime = getRuntime();
            firstRunShootThreeRings = false;
        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        if (currentTime - startShootingTime  < 1) {
            ShooterFeedingServo.setPosition(launchPosition);
        }
        else if (currentTime - startShootingTime  < 2) {
            ShooterFeedingServo.setPosition(storePosition);
        }
        else if (currentTime - startShootingTime  < 3) {
            ShooterFeedingServo.setPosition(launchPosition);
        }
        else if (currentTime - startShootingTime  < 4) {
            ShooterFeedingServo.setPosition(storePosition);
        }
        else if (currentTime - startShootingTime  < 5) {
            ShooterFeedingServo.setPosition(launchPosition);
        }
        else if (currentTime - startShootingTime  < 6) {
            ShooterFeedingServo.setPosition(storePosition);
        }
        else if (currentTime - startShootingTime  < 7) {
            ShooterFeedingServo.setPosition(launchPosition);
        }
        else if (currentTime - startShootingTime  < 8) {
            ShooterFeedingServo.setPosition(storePosition);
        }
        else {
            TransferMotor.setTargetPosition(0);
        }

        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
    }

    //WOBBLE ---------------------------------------------------------------------------------------

    private void initializeWobble() {
        WobbleMotor = hardwareMap.dcMotor.get("WobbleMotor");
        WobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WobbleMotor.setTargetPosition(0);
        WobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleMotor.setPower(.5);
    }
    private void startWobble() {
        WobbleMotor.setTargetPosition(100);
    }

    private void dumpRings(int thisState, int nextState, double timeout) {
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        WobbleMotor.setTargetPosition(350);

        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
    }

    private void raiseRings(int thisState, int nextState, double timeout) {
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        WobbleMotor.setTargetPosition(-5);

        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
    }

    private void dropWobbleGoal(int thisState, int nextState, double timeout) {
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        WobbleMotor.setTargetPosition(685);

        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
    }
}

