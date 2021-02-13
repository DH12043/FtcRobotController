package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "motortesting")
public class motortesting extends OpMode{
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;

    double startTime;
    double currentTime;

    @Override
    public void init() {
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        startTime = getRuntime();
    }

    @Override
    public void loop() {
        currentTime = getRuntime();

        if (currentTime-startTime < 2) {
            FrontLeft.setPower(1);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontRight.setPower(0);
        }
        else if (currentTime-startTime < 4) {
            FrontLeft.setPower(0);
            BackLeft.setPower(1);
            BackRight.setPower(0);
            FrontRight.setPower(0);
        }
        else if (currentTime-startTime < 6) {
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(1);
            FrontRight.setPower(0);
        }
        else if (currentTime-startTime < 8) {
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontRight.setPower(1);
        }
    }
}
