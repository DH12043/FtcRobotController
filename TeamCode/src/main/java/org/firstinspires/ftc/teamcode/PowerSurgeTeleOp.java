package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.Range;
import java.io.File;

@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    //ODOMETRY/DRIVETRAIN/IMU VARIABLES AND DEVICES ------------------------------------------------

    public static final double DEADZONE = 0.15;

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

    private double ScoringXPosition;
    private double ScoringYPosition;
    private double ScoringRotation;

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    final double COUNTS_PER_INCH = 307.699557;

    private static final double DECELERATION_START_POINT = 24;
    private static final double DECELERATION_ZERO_POINT = -1;
    private static final double TURNING_DECELERATION_START_POINT = 100;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1;

    private double lastDistanceToTarget = 0;
    private double distanceToTarget;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;
    //DISTANCE SENSORS -----------------------------------------------------------------------------

    private ModernRoboticsI2cRangeSensor LeftDistanceSensor;
    private ModernRoboticsI2cRangeSensor BackDistanceSensor;
    private ModernRoboticsI2cRangeSensor RightDistanceSensor;

    private double leftDistance;
    private double backDistance;
    private double rightDistance;

    //SHOOTER VARIABLES ----------------------------------------------------------------------------

    private boolean firstPressShooterToggleButton = true;
    private boolean firstPressShooterFeedingToggleButton = true;
    private boolean firstPressShooterSpeedToggleButton = true;
    private boolean shooterOn = false;
    private boolean shooterIsFast = true;
    private boolean shooterFeedingOn = false;
    private boolean bucketUp = false;
    private boolean firstPressTransferToggleButton = false;

    private double shooterTicksPerRevolution = 103.6;

    private int currentShooterTickCount;
    private int previousShooterTickCount;

    //INTAKE VARIABLES -----------------------------------------------------------------------------

    private boolean firstPressIntakeToggleButton = true;
    private boolean intakeOn = false;

    //Wobble Variables -----------------------------------------------------------------------------
    private boolean firstPressWobbleButton = true;
    private boolean wobbleUp = true;
    private boolean slowDriveSpeed = false;
    private int globalWobbleOffset = 0;
    private boolean wobbleHasMoved = false;

    //GAMEPAD IMPUTS -------------------------------------------------------------------------------

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private boolean shooterToggleButton;
    private boolean shooterFeedingServoButton;
    private boolean shooterSpeedToggleButton;
    private boolean transferToggleButton;
    private boolean intakeToggleButton;
    private boolean driveToLaunchButton;
    private boolean wobbleButton;
    private boolean wobbleOffsetUp;
    private double wobbleOffsetDown;

   //MOTORS AND SERVOS -----------------------------------------------------------------------------

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;

    private DcMotor IntakeMotor;
    private DcMotor ShooterMotor;
    private DcMotor TransferMotor;

    private DcMotor WobbleMotor;

    private Servo ShooterFeedingServo;



    //LPS COUNTER ----------------------------------------------------------------------------------

    private int loopCount;
    private double loopStartTime;
    private double loopsPerSecond;

    private double currentTime;
    private double previousTime;

    private long lastUpdateTime = 0;

    private double startShooterFeedingTime;

    @Override
    public void init() {
        telemetry.addData("Version Number", "12/22/20");
        initializeDriveTrain();
        initializeOdometry();
        initializeIMU();
        initializeShooter();
        initializeIntake();
        initializeWobble();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startIMU();
        startOdometry();
        startWobble();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }

    @Override
    public void loop() {
        double currentTimeAtCheckLPS = getRuntime();
        oldCheckLPS();

        driveToLaunchButton = gamepad1.right_bumper;

        if(driveToLaunchButton) {
            goToPosition(0, 0, 1, 1, 0);
        }
        else if(slowDriveSpeed) {
            movement_y = .5 * DeadModifier(-gamepad1.left_stick_y);
            movement_x = .5 * DeadModifier(gamepad1.left_stick_x);
            movement_turn = .5 * DeadModifier(.75 * gamepad1.right_stick_x);
        }
        else {
            movement_y = DeadModifier(-gamepad1.left_stick_y);
            movement_x = DeadModifier(gamepad1.left_stick_x);
            movement_turn = DeadModifier(.75 * gamepad1.right_stick_x);
        }

        shooterToggleButton = gamepad1.a;
        shooterFeedingServoButton = gamepad1.x;
        transferToggleButton = gamepad1.b;
        wobbleButton = gamepad1.dpad_up;
        shooterSpeedToggleButton = gamepad1.y;
        wobbleOffsetUp = gamepad1.left_bumper;
        wobbleOffsetDown = gamepad1.left_trigger;

        applyMovement();
        checkOdometry();
        runShooter();
        //runIntake();
        runWobble();

        telemetry.update();
    }

    @Override
    public void stop() {
        if (globalPositionUpdate != null) {
            globalPositionUpdate.stop();
        }
    }

    //DRIVETRAIN -----------------------------------------------------------------------------------

    private double DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
    }

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

        double fl_power_raw = movement_y+movement_turn+movement_x;
        double bl_power_raw = movement_y+movement_turn-movement_x;
        double br_power_raw = -movement_y+movement_turn-movement_x;
        double fr_power_raw = -movement_y+movement_turn+movement_x;

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
        FrontLeft.setPower(fl_power_raw);
        BackLeft.setPower(-bl_power_raw);
        BackRight.setPower(br_power_raw);
        FrontRight.setPower(fr_power_raw);
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
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //StartingXPosition = Double.parseDouble(ReadWriteFile.readFile(startingXpositionFile).trim());
        //StartingYPosition = Double.parseDouble(ReadWriteFile.readFile(startingYpositionFile).trim());
        //StartingRotation = Double.parseDouble(ReadWriteFile.readFile(startingθpositionFile).trim());
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 0;

    }

    private void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 25);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    private void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
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

    //goToPosition ---------------------------------------------------------------------------------

    private void goToPosition(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget
                - Math.toRadians(RobotRotation) + Math.toRadians(90));

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

        if (distanceToTarget < 1) {
            movement_turn = 0;
        } else {
            //movement_turn = Range.clip(Range.clip(relativeTurnAngle / Math.toRadians(30),
            //        -1, 1) * maxTurnSpeed, -turnDecelLimiter, turnDecelLimiter);
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

        composeTelemetry();
    }

    private void startIMU() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //imu2.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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
        ShooterFeedingServo.setPosition(.6);

        TransferMotor = hardwareMap.dcMotor.get("TransferMotor");
        TransferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TransferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TransferMotor.setTargetPosition(0);
        TransferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TransferMotor.setPower(.2);
    }

    private void runShooter() {
        if(shooterSpeedToggleButton) {
            if (firstPressShooterSpeedToggleButton) {
                if (shooterIsFast) {
                    shooterIsFast = false;
                }
                else {
                    shooterIsFast = true;
                }
                firstPressShooterSpeedToggleButton = false;
            }
        }
        else {
            firstPressShooterSpeedToggleButton = true;
        }

        if (shooterToggleButton) {
            if (firstPressShooterToggleButton) {
                if (shooterOn) {
                    shooterOn = false;
                }
                else {
                    shooterOn = true;
                }
                firstPressShooterToggleButton = false;
            }
        }
        else {
            firstPressShooterToggleButton = true;
        }

        if (shooterOn) {
            if (shooterIsFast) {
                ShooterMotor.setPower(.7); // was .8 before adding mass  //.6 for power shots
            }
            else {
                ShooterMotor.setPower(.25);
            }
        }
        else {
            ShooterMotor.setPower(0);
        }

        if (transferToggleButton) {
            if (firstPressTransferToggleButton) {
                bucketUp = !bucketUp;
                firstPressTransferToggleButton = false;
            }
        }
        else {
            firstPressTransferToggleButton = true;
        }

        if (bucketUp) {
            if (shooterFeedingServoButton) {
                ShooterFeedingServo.setPosition(.2);
            }
            else{
                ShooterFeedingServo.setPosition(.6);
            }
            TransferMotor.setTargetPosition(-140);
            IntakeMotor.setPower(0);

        }
        else {
            ShooterFeedingServo.setPosition(.6);
            TransferMotor.setTargetPosition(0);
            IntakeMotor.setPower(-.5);
        }

        currentShooterTickCount = ShooterMotor.getCurrentPosition();

        double shooterRevolutionsPerLoop = ((currentShooterTickCount - previousShooterTickCount) / shooterTicksPerRevolution);
        double shooterRevolutionsPerMinute = shooterRevolutionsPerLoop * loopsPerSecond *60*3;

        telemetry.addData("shooterRevolutionsPerMinute", shooterRevolutionsPerMinute);

        previousShooterTickCount = currentShooterTickCount;

    }


    //INTAKE ---------------------------------------------------------------------------------------

    private void initializeIntake() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void runIntake() {
        if (intakeToggleButton) {
            if (firstPressIntakeToggleButton) {
                if (intakeOn) {
                    IntakeMotor.setPower(0);
                    intakeOn = false;
                }
                else {
                    IntakeMotor.setPower(-.5);
                    intakeOn = true;
                }
                firstPressIntakeToggleButton = false;
            }
        }
        else {
            firstPressIntakeToggleButton = true;
        }
    }

    //Wobble ---------------------------------------------------------------------------------------

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

    private void runWobble() {
        if (wobbleOffsetUp == true) {
            globalWobbleOffset = globalWobbleOffset - 3;
        }
        else if (wobbleOffsetDown > .5) {
            globalWobbleOffset = globalWobbleOffset + 3;
        }

        if (wobbleButton) {
            if (firstPressWobbleButton) {
                if (wobbleUp) {
                    WobbleMotor.setPower(.5);
                    //WobbleMotor.setTargetPosition(675 + globalWobbleOffset);
                    slowDriveSpeed = true;
                    wobbleUp = false;
                }
                else {
                    WobbleMotor.setPower(.2);
                    //WobbleMotor.setTargetPosition(200 + globalWobbleOffset);
                    slowDriveSpeed = false;
                    wobbleUp = true;
                }
                firstPressWobbleButton = false;
            }
            wobbleHasMoved = true;
        }
        else {
            firstPressWobbleButton = true;
        }

        if (wobbleHasMoved) {
            if (wobbleUp) {
                WobbleMotor.setTargetPosition(200 + globalWobbleOffset);
            } else {
                WobbleMotor.setTargetPosition(685 + globalWobbleOffset);
            }
        }

        telemetry.addData("Wobble Position", WobbleMotor.getCurrentPosition());
        telemetry.addData("GlobalWobbleOffset", globalWobbleOffset);
    }


    //Check LPS ------------------------------------------------------------------------------------

    private void checkLPS() {
        currentTime = getRuntime();

        double timePerLoop = currentTime - previousTime;
        loopsPerSecond = 1 / timePerLoop;

        previousTime = currentTime;
    }

    private void oldCheckLPS() {
        double currentTime = getRuntime();
        loopCount++;
        if (currentTime > loopStartTime + 1) {
            loopsPerSecond = loopCount;
            loopCount = 0;
            loopStartTime = currentTime;
        }
        telemetry.addData("LPS: ", loopsPerSecond);
    }

}
