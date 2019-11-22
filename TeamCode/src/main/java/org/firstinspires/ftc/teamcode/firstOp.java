package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class firstOp extends LinearOpMode {


    private DcMotor right_drive;
    private Servo testservo;
    private Servo testservo2;
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
    private DcMotor left_drive;
    private DcMotor testarmmotor;

    @Override
    public void runOpMode() {
        float armpower;
        int colorHSV;
        float hue;
        float sat;
        float val;

        right_drive = hardwareMap.dcMotor.get("right_drive");
        testservo = hardwareMap.servo.get("test servo");
        testservo2 = hardwareMap.servo.get("test servo2");
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.colorSensor.get("sensorColorRange");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        testarmmotor = hardwareMap.dcMotor.get("test arm motor");
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        testservo.setPosition(0.5);
        testservo2.setPosition(0.5);

        telemetry.addData("Color Distance", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
                telemetry.addData("Light detected", ((OpticalDistanceSensor) sensorColorRange_REV_ColorRangeSensor).getLightDetected());
                colorHSV = Color.argb(sensorColorRange_REV_ColorRangeSensor.alpha(), sensorColorRange_REV_ColorRangeSensor.red(), sensorColorRange_REV_ColorRangeSensor.green(), sensorColorRange_REV_ColorRangeSensor.blue());
                hue = JavaUtil.colorToHue(colorHSV);
                telemetry.addData("Hue", hue);
                sat = JavaUtil.colorToSaturation(colorHSV);
                telemetry.addData("Saturation", sat);
                val = JavaUtil.colorToValue(colorHSV);
                telemetry.addData("Value", val);
                if (hue < 30) {
                    telemetry.addData("Color", "Red");
                } else if (hue < 60) {
                    telemetry.addData("Color", "Orange");
                } else if (hue < 90) {
                    telemetry.addData("Color", "Yellow");
                } else if (hue < 150) {
                    telemetry.addData("Color", "Green");
                } else if (hue < 225) {
                    telemetry.addData("Color", "Blue");
                } else if (hue < 350) {
                    telemetry.addData("Color", "purple");
                } else {
                    telemetry.addData("Color", "Red");
                }
                if (sat < 0.2) {
                    telemetry.addData("Check Sat", "Is surface white?");
                }
                telemetry.update();
                if (val < 0.16) {
                    telemetry.addData("Check Val", "Is surface black?");
                }
                if (gamepad1.left_bumper) {
                    testservo.setPosition(0);
                    testservo2.setPosition(1);
                } else if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
                    testservo.setPosition(0.5);
                    testservo2.setPosition(0.5);
                } else if (gamepad1.right_bumper) {
                    testservo.setPosition(1);
                    testservo2.setPosition(0);
                }
                left_drive.setPower(gamepad1.left_stick_y);
                right_drive.setPower(gamepad1.right_stick_y);
                armpower = gamepad1.right_stick_x;
                testarmmotor.setPower(armpower);
                telemetry.addData("Left Power", left_drive.getPower());
                telemetry.addData("Right Power", right_drive.getPower());
                telemetry.addData("Servo Position", testservo.getPosition());
                telemetry.addData("Servo Position", testservo2.getPosition());
                telemetry.addData("Arm Power", armpower);
                telemetry.update();
            }
        }
    }
}

