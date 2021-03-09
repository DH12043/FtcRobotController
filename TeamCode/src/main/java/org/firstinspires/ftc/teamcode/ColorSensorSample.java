package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorSample extends OpMode {

    NormalizedColorSensor transferColorSensor;

    final float[] hsvValues = new float[3];

    @Override
    public void init() {
        transferColorSensor = hardwareMap.get(NormalizedColorSensor.class, "transferColorSensor");
    }

    @Override
    public void loop() {
        NormalizedRGBA transferColors = transferColorSensor.getNormalizedColors();

        Color.colorToHSV(transferColors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", transferColors.red)
                .addData("Green", "%.3f", transferColors.green)
                .addData("Blue", "%.3f", transferColors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", transferColors.alpha);

        telemetry.update();
    }

}