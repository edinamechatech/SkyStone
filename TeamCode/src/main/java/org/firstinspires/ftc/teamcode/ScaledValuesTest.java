package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class ScaledValuesTest extends LinearOpMode {

    //elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //name the different variables for the motors/servos
    private DcMotor right_front = null;
    private DcMotor right_back = null;
    private DcMotor left_front = null;
    private DcMotor left_back = null;
    private DcMotor armmotor;
    private Servo armservo1;
    private Servo armservo2;
    private Servo servo;

    @Override
    public void runOpMode() {

        //set the movement variables for mecanum wheels
        double vertical;
        double horizontal;
        double pivot;
        //set variable for armpower
        double armpower;

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
        servo = hardwareMap.servo.get("servo");

        //set motors to run using encoders
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set servo positions to start position (all raised and compact)
        armservo1.setPosition(0.0);
        armservo2.setPosition(1.0);
        servo.setPosition(0.0);

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

            //set and scale the mecanum wheel variables
            vertical = -((gamepad1.right_stick_y / 1.20) * (.62 * (gamepad1.right_stick_y * gamepad1.right_stick_y)) + .20);
            if(gamepad1.right_stick_y < 0) {
                vertical = -((gamepad1.right_stick_y / 1.20) * (.62 * (gamepad1.right_stick_y * gamepad1.right_stick_y)) - .20);
            }
            if (gamepad1.right_stick_y == 0) {
                vertical = 0;
            }
            if (gamepad1.right_stick_y > 0 && gamepad1.right_stick_y < .45 || gamepad1.right_stick_y < 0 && gamepad1.right_stick_y > -.45) {
                vertical = -gamepad1.right_stick_y / 1.20;
            }


            horizontal = -((gamepad1.right_stick_x / 1.20) * (.62 * (gamepad1.right_stick_x * gamepad1.right_stick_x)) + .20);
            if(gamepad1.right_stick_x < 0) {
                horizontal = -((gamepad1.right_stick_x / 1.20) * (.62 * (gamepad1.right_stick_x * gamepad1.right_stick_x)) - .20);
            }
            if (gamepad1.right_stick_x == 0) {
                horizontal = 0;
            }
            if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_x < .45 || gamepad1.right_stick_x < 0 && gamepad1.right_stick_x > -.45) {
                horizontal = -gamepad1.right_stick_x / 1.20;
            }


            pivot = ((gamepad1.left_stick_x / 1.20) * (.62 * (gamepad1.left_stick_x * gamepad1.left_stick_x)) + .20);
            if (gamepad1.left_stick_x < 0 ) {
                pivot = ((gamepad1.left_stick_x / 1.20) * (.62 * (gamepad1.left_stick_x * gamepad1.left_stick_x)) - .20);
            }
            if (gamepad1.left_stick_x == 0) {
                pivot = 0;
            }
            if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_x < .45 || gamepad1.left_stick_x < 0 && gamepad1.left_stick_x > -.45) {
                pivot = gamepad1.left_stick_x / 1.20;
            }


           //set the power for the wheel motors to the variables listed above
            right_front.setPower(-pivot + (vertical - horizontal));
            right_back.setPower(-pivot + vertical + horizontal);
            left_front.setPower(pivot + vertical + horizontal);
            left_back.setPower(pivot + (vertical - horizontal));

            //set the armpower to scaled left_stick_y
            armpower = (gamepad1.left_stick_y / 1.07);
            if (gamepad1.left_stick_y < 0) {
                armpower = (gamepad1.left_stick_y / 1.07);
            }
            armmotor.setPower(armpower);

            //bumpers on gamepad control how the servo moves
            if (gamepad1.left_bumper) {
                armservo1.setPosition(0);
                armservo2.setPosition(1);
                //mainly testing for starting position
            } else if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
                armservo1.setPosition(0.5);
                armservo2.setPosition(0.5);
            } else if (gamepad1.right_bumper) {
                armservo1.setPosition(1);
                armservo2.setPosition(0);
            }
            //left and right triggers on gamepad control how side servo moves
            if (gamepad1.a) {
                servo.setPosition(0);
            }
            if (gamepad1.b) {
                servo.setPosition(1);
            }


            //send the data over to the robot
            telemetry.addData("RF Power", right_front.getPower());
            telemetry.addData("RB Power", right_back.getPower());
            telemetry.addData("LF Power", left_front.getPower());
            telemetry.addData("LB Power", left_back.getPower());
            telemetry.addData("Arm Power", armmotor.getPower());
            telemetry.addData("Left Claw Position", armservo1.getPosition());
            telemetry.addData("Right Claw Position", armservo2.getPosition());
            telemetry.addData("SideServo", servo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //update the data on the robot
            telemetry.update();
        }
    }
}
