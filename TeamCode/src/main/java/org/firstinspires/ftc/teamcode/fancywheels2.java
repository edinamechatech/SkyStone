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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="fancywheels", group="Linear Opmode")
@Disabled
public class fancywheels2 extends LinearOpMode {

    //elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //name the different variables for the motors
    private DcMotor right_front = null;
    private DcMotor right_back = null;
    private DcMotor left_front = null;
    private DcMotor left_back = null;

    @Override
    public void runOpMode() {

        //set the movement variables for mecanum wheels
        double vertical;
        float horizontal;
        float pivot;

        //get the motors
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

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

            //set the mecanum wheel variables to gamepad joysticks
            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;
            //set the power for the wheel motors to the variables listed above
            right_front.setPower(-pivot + (vertical - horizontal));
            right_back.setPower(-pivot + vertical + horizontal);
            left_front.setPower(pivot + vertical + horizontal);
            left_back.setPower(pivot + (vertical - horizontal));
            //send the data over to the robot
            telemetry.addData("Right Front Power", right_front.getPower());
            telemetry.addData("Right Back Power", right_back.getPower());
            telemetry.addData("Left Front Power", left_front.getPower());
            telemetry.addData("Left Back Power", left_back.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //update the data on the robot
            telemetry.update();

        }
    }
}
