package org.firstinspires.ftc.teamcode;

//IN ORDER TO TRANSFER CODE CONNECT THE ROBOT CONTROLLER PHONE TO YOUR LAPTOP ANDPRESS RUN AT THE TOP
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class fancywheels2 extends LinearOpMode {

    //elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //name the different variables for the motors
    private DcMotor right_front = null;
    private DcMotor right_back = null;
    private DcMotor left_front = null;
    private DcMotor left_back = null;
    private DcMotor armmotor;
    private Servo armservo1;
    private Servo armservo2;
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;


    @Override
    public void runOpMode() {

        //set the movement variables for mecanum wheels
        double vertical;
        float horizontal;
        float pivot;
        //set variable for armpower
        float armpower;
        //set variables for color sensor
        int colorHSV;
        float hue;
        float sat;
        float val;


        //get the motors
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        //get armmotor
        armmotor = hardwareMap.dcMotor.get("armmotor");
        //get the servos
        armservo1 = hardwareMap.servo.get("armservo1");
        armservo2 = hardwareMap.servo.get("armservo2");
        //get color sensor
//        sensorColorRange_REV_ColorRangeSensor = hardwareMap.colorSensor.get("sensorColorRange");


        //set mode of motors initially to using encoders
//        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set servo positions to start position
        armservo1.setPosition(0.5);
        armservo2.setPosition(0.5);

        //set the right side wheels to reverse so all wheels move in the same direction--not flipped directions
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        //to see if the teleOp is actually working before starting
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //add data of color sensors to the robot--commented out code is mainly for testing
//            telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
//            telemetry.addData("Light detected", ((OpticalDistanceSensor) sensorColorRange_REV_ColorRangeSensor).getLightDetected());
//            colorHSV = Color.argb(sensorColorRange_REV_ColorRangeSensor.alpha(), sensorColorRange_REV_ColorRangeSensor.red(), sensorColorRange_REV_ColorRangeSensor.green(), sensorColorRange_REV_ColorRangeSensor.blue());
//            hue = JavaUtil.colorToHue(colorHSV);
//            telemetry.addData("Hue", hue);
//            sat = JavaUtil.colorToSaturation(colorHSV);
//            telemetry.addData("Saturation", sat);
//            val = JavaUtil.colorToValue(colorHSV);
//            telemetry.addData("Value", val);

            //what color is being detected?
//            if (hue < 30) {
//                telemetry.addData("Color", "Red");
//            } else if (hue < 60) {
//                telemetry.addData("Color", "Orange");
//            } else if (hue < 90) {
//                telemetry.addData("Color", "Yellow");
//            } else if (hue < 150) {
//                telemetry.addData("Color", "Green");
//            } else if (hue < 225) {
//                telemetry.addData("Color", "Blue");
//            } else if (hue < 350) {
//                telemetry.addData("Color", "purple");
//            } else {
//                telemetry.addData("Color", "Red");
//            }
//
//            //saturation testing
//            if (sat < 0.2) {
//                telemetry.addData("Check Sat", "Is surface white?");
//            }
//            telemetry.update();
//            if (val < 0.16) {
//                telemetry.addData("Check Val", "Is surface black?");
//            }

            //set the mecanum wheel variables to gamepad joysticks
            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;
            //set the power for the wheel motors to the variables listed above
            right_front.setPower(-pivot + (vertical - horizontal));
            right_back.setPower(-pivot + vertical + horizontal);
            left_front.setPower(pivot + vertical + horizontal);
            left_back.setPower(pivot + (vertical - horizontal));

            //set the armpower to left_stick_y
            armpower = gamepad1.left_stick_y;
            armmotor.setPower(armpower);

            //bumpers on gamepad control how the servo moves
            if (gamepad1.left_bumper) {
                armservo1.setPosition(0);
                armservo2.setPosition(1);
                //mainly testing for starting position
//            } else if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
//                armservo1.setPosition(0.5);
//                armservo2.setPosition(0.5);
            } else if (gamepad1.right_bumper) {
                armservo1.setPosition(1);
                armservo2.setPosition(0);
            }

            //send the data over to the robot
            telemetry.addData("Right Front Power", right_front.getPower());
            telemetry.addData("Right Back Power", right_back.getPower());
            telemetry.addData("Left Front Power", left_front.getPower());
            telemetry.addData("Left Back Power", left_back.getPower());
            telemetry.addData("Arm Power", armpower);
            telemetry.addData("Servo Position", armservo1.getPosition());
            telemetry.addData("Servo Position", armservo2.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //update the data on the robot
            telemetry.update();

        }
    }
}
