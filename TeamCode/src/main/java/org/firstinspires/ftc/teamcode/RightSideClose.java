/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RightSideClose")

//    @TeleOp
public class RightSideClose extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 280;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    //name the different variables for the motors/servos/sensors
    private DcMotor right_front = null;
    private DcMotor right_back = null;
    private DcMotor left_front = null;
    private DcMotor left_back = null;

    private DcMotor armmotor;
    private Servo armservo1;
    private Servo armservo2;
    private Servo servo;
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;

    @Override
    public void runOpMode() {

        double vertical;
        float horizontal;
        float pivot;


        armservo1 = hardwareMap.servo.get("armservo1");
        armservo2 = hardwareMap.servo.get("armservo2");
        servo = hardwareMap.servo.get("servo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        armservo1.setPosition(0.0);
        armservo2.setPosition(1.0);
        servo.setPosition(0.0);

        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          right_back.getCurrentPosition(), right_front.getCurrentPosition(), left_back.getCurrentPosition(), left_front.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  -4, 4,4,-4,1);  // S1: Strafe left 2 Inches with 3 Sec timeout
        encoderDrive(DRIVE_SPEED,  1,1 , 1,1,1);  // S2: Forward 26 Inches with 3 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double rightFrontInches, double leftFrontInches, double rightBackInches, double leftBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("Right Front Power", right_front.getPower());
            telemetry.addData("Right Back Power", right_back.getPower());
            telemetry.addData("Left Front Power", left_front.getPower());
            telemetry.addData("Left Back Power", left_back.getPower());
            // Determine new target position, and pass to motor controller
            newLeftTarget = left_front.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightTarget = right_front.getCurrentPosition() - (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightTarget2 = right_back.getCurrentPosition() - (int)(rightBackInches * COUNTS_PER_INCH);
            newLeftTarget2 = left_back.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);

            left_front.setTargetPosition(newLeftTarget);
            left_back.setTargetPosition(newLeftTarget2);
            right_front.setTargetPosition(newRightTarget);
            right_back.setTargetPosition(newRightTarget2);


            // Turn On RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_front.setPower(Math.abs(speed));
            left_back.setPower(Math.abs(speed));
            right_front.setPower(Math.abs(speed));
            right_back.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) && (left_front.isBusy() && right_front.isBusy() && left_back.isBusy() && right_back.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget, newRightTarget2, newLeftTarget2);
                telemetry.addData("Path2",  "Running at %7d :%7d", left_front.getCurrentPosition(), right_front.getCurrentPosition(), left_back.getCurrentPosition(), right_back.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            right_back.setPower(0);
            right_front.setPower(0);
            left_back.setPower(0);
            left_front.setPower(0);


            // Turn off RUN_TO_POSITION
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}
