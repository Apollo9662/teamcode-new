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

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MECK_TEST", group="APOLLO")
@Disabled
public class mech_test extends LinearOpMode {

    int speedFactor = 1;
    Hardware robot           = new Hardware();   // Use a Pushbot's hardware


    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,true);



        double right_x = gamepad1.right_stick_x;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double left_y = gamepad1.left_stick_y;




        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();

        // run until the en of the match (driver presses STOP)
        while (opModeIsActive()) {
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            right_x = gamepad1.right_stick_x;
            right_y = -gamepad1.right_stick_y;

            if (left_x < -0.05 && Math.abs(left_y) > 0.05 && right_x < -0.05 && Math.abs(right_y) > 0.05){
                robot.setDriveMotorsPower(left_y*speedFactor, Hardware.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                telemetry.addData("","DIAGONAL_LEFT");

            }
            else if (left_x > 0.05 && Math.abs(left_y) >0.05 && right_x > 0.05 && Math.abs(right_y) > 0.05){
                robot.setDriveMotorsPower(left_y*speedFactor, Hardware.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                telemetry.addData("","DIAGONAL_RIGHT");

            }
            else if (left_x <-0.05 && right_x <-0.05) {
                robot.setDriveMotorsPower(left_x, Hardware.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("","LEFT");

            }
            else if (left_x > 0.05 && right_x >0.05) {
                robot.setDriveMotorsPower(left_x, Hardware.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("","RIGHT");

            }
            else{
               robot.setDriveMotorsPower(left_y, Hardware.DRIVE_MOTOR_TYPES.LEFT);
               robot.setDriveMotorsPower(right_y, Hardware.DRIVE_MOTOR_TYPES.RIGHT);
                telemetry.addData("","TANK");


            }
            telemetry.addData("X position",robot.getX());
            telemetry.addData("Y position",robot.getY());
            telemetry.update();
        }
    }
}
