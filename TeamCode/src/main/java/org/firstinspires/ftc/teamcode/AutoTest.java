package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode{

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
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "FrontRight";

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;

    private double StartingXPosition;
    private double StartingYPosition;
    private double StartingRotation;

    final double COUNTS_PER_INCH = 307.699557;

    private static final double DECELERATION_START_POINT = 24;
    private static final double DECELERATION_ZERO_POINT = -6;
    private static final double TURNING_DECELERATION_START_POINT = 100;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1;

    private double lastDistanceToTarget = 0;
    private double distanceToTarget;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private int loopCount;
    private double loopStartTime;
    private int loopsPerSecond;

    private long lastUpdateTime = 0;

    @Override
    public void init() {
        initVuforia();
        initTfod();

        initializeDriveTrain();
        initializeOdometry();
        initializeIMU();

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
    public void start() {
        if (tfod != null) {
            tfod.shutdown();
        }

        startIMU();
        startOdometry();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }
    public void loop() {
        double currentTime = getRuntime();
        loopCount++;
        if (currentTime > loopStartTime + 5) {
            loopsPerSecond = loopCount;
            loopCount = 0;
            loopStartTime = currentTime;
        }
        telemetry.addData("LPS: ", loopsPerSecond);

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
                    if (recognition.getLabel().equals("Single")) {
                        path = 1;
                    }

                }
            }
        }
        else {
            path = 0;
        }

        telemetry.addData("Path", path);

        applyMovement();
        checkOdometry();

        telemetry.update();

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
        double br_power_raw = -movement_y+movement_turn+movement_x;
        double fr_power_raw = -movement_y+movement_turn-movement_x;

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
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 25, imu);
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

}
