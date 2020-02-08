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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
import static org.firstinspires.ftc.teamcode.MathFunctions.*;


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

@TeleOp(name="TeleopN", group="APOLLO")
public class TeleopNew extends functions {

    private Thread slidesOperation = new slidesOperation();
    private Thread clawOperation = new clawOperation();
    private Thread catchers = new catchers();
    private Thread inputOperation = new InputOperation();
    private double collectionSpeed = 1;
    private int[] levels = {0,6,10,15,18,23};
    boolean verticalMotorOnTarget = true;
    double turnPower = 0;
    double errorKeap;
    double robotAngle = 0;
    boolean CAPSTONE = false;


    private static final double countsPer20gearmotor = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.4;     // This is < 1.0 if geared UP
    private static final double pulleyDiameter = 1;     // For figuring circumference
    private static final double COUNTS_PER_INCH_FOR_VERTICAL_SLIDE = (countsPer20gearmotor / DRIVE_GEAR_REDUCTION) / (Math.PI);


    private static final long threadSleepTimeOut = 100; // 50 msec

    private static final double ticksPerLevel = 30.0;
    private static final int maxLevel = 7;
    private static final double maxElevatorValue = countsPer20gearmotor * (ticksPerLevel * maxLevel);

    private int currentLevel = 0;
    private double elevatorHeight = 0;
    private double driftSpeed = 0.4;
    private boolean fail = false;

    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("imu is working ==> ", robot.imuIsWorking);
        telemetry.addData("imu angle ==> ", robot.GetGyroAngle());
        telemetry.update();

        waitForStart();
        clawOperation.start();
        slidesOperation.start();
        inputOperation.start();
        catchers.start();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try {
                robot.updatePosition(true);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            telemetry.addData("vertical", robot.finalAngle);
            telemetry.addData("x", robot.leftCollector.getCurrentPosition());
            telemetry.addData("y", robot.rightCollector.getCurrentPosition());
            telemetry.addData("dist",robot.sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("stone level", currentLevel);
            telemetry.addData("elevator height", elevatorHeight);
            telemetry.addData("turn speed", turnPower);
            telemetry.addData("robot real angle", robotAngle);
            telemetry.addData("robot error angle", errorKeap);
            telemetry.addData("robot x", robot.robotPosition.x);
            telemetry.addData("robot y", robot.robotPosition.y);
            telemetry.update();
            driveOperation();
        }
    }


    private class clawOperation extends Thread {
        clawOperation() {
            this.setName("clawOperation");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {
                    while (!isInterrupted() && opModeIsActive()) {

                        double backPosition;
                        double frontPosition;
                        if (!CAPSTONE) {
                            if (gamepad2.left_trigger > 0.1) {
                                frontPosition = frontOpenPos;
                            } else {
                                frontPosition = frontClosePos;
                            }

                            if (gamepad2.right_trigger > 0.1) {
                                //telemetry.addData("ga", gamepad2.right_trigger);
                                backPosition = backOpenPos;
                            } else {
                                backPosition = backClosePos;
                            }
                            if (gamepad1.left_trigger > 0.1) {
                                frontPosition = frontOpenPos;
                            }
                            if (gamepad2.b) {
                                fail = true;
                            } else if (gamepad2.x) {
                                fail = false;

                            }

                            if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 6) {
                                frontPosition = frontClosePos;
                                backPosition = backClosePos;

                            }

                            if (fail) {
                                frontPosition = frontClosePos;
                                backPosition = backClosePos;
                            }
                        } else {
                            frontPosition = frontOpenPos;
                            backPosition = backOpenPos;
                        }


                        robot.frontClaw.setPosition(frontPosition);
                        robot.backClaw.setPosition(backPosition);

                        Thread.sleep(threadSleepTimeOut);
                        if (gamepad2.y) {
                            if (CAPSTONE) {
                                robot.capStone.setPosition(0);
                                CAPSTONE = false;
                            } else {
                                robot.capStone.setPosition(0.2);
                                CAPSTONE = true;
                            }

                        }
                    }
                }
            } catch (InterruptedException ignored) {
            }
        }
    }


    private class slidesOperation extends Thread {
        slidesOperation() {
            this.setName("slidesOperation");
        }
        @Override
        public void run() {
            try {
                boolean motorOnTarget = true;
                robot.verticalElevator.setTargetPosition(0);
                while (!isInterrupted() && opModeIsActive()) {

                    robot.verticalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.verticalElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.verticalElevator.setPower(-gamepad2.left_stick_y);
                    robot.horizontalElevator.setPower(gamepad2.right_stick_x);
                        elevatorHeight = robot.verticalElevator.getCurrentPosition() / COUNTS_PER_INCH_FOR_VERTICAL_SLIDE;
                        if (gamepad2.left_bumper) {
                            if (currentLevel < 5) {
                                currentLevel = currentLevel + 1;
                            }
                            while (gamepad2.left_bumper && opModeIsActive()) {
                                Thread.sleep(threadSleepTimeOut);
                            }
                        }
                        if (gamepad2.right_bumper) {

                            if (currentLevel > 0) {
                                currentLevel = currentLevel - 1;
                            }
                            while (gamepad2.left_bumper && opModeIsActive()) {
                                Thread.sleep(threadSleepTimeOut);
                            }

                        }

                        robot.AutoVerticalSlide(currentLevel, gamepad2.a);
                    }

//                   }else if (robot.verticalElevator.getCurrentPosition() > robot.verticalElevator.getTargetPosition() && opModeIsActive()){
//                       robot.verticalElevator.setPower(-verticalSpeed);
//                   }
                    Thread.sleep(threadSleepTimeOut);
            } catch (InterruptedException ignored) {
            }
        }
    }


    public void driveOperation() {
        turnPower = -gamepad1.right_stick_x * 0.5;
        double right_x = -gamepad1.left_stick_x;
        double right_y = gamepad1.left_stick_y;

        double angle = Math.toDegrees(Math.atan2(right_y, right_x));
        angle += 135;
        right_y = -right_y;
        telemetry.addData("angle = ",angle);
        if(abs(right_x)>0.1 && abs(right_y)>0.1){
            turnPower = -gamepad1.right_stick_x;
        }
        if(!inRange(angle,20,70) && !inRange(angle,200,250) ){
            if (abs(turnPower) < 0.03 && abs(right_x)>0.05 && abs(right_y)>0.05) {
                double DriveError = getError(robotAngle);
                turnPower = getSteer(DriveError, 0.01);
            } else {
                robotAngle = robot.GetGyroAngle();
            }
            robot.driveLeftFront.setPower(-right_x * driftSpeed - turnPower);
            robot.driveRightBack.setPower(-right_x * driftSpeed + turnPower);
            robot.driveLeftBack.setPower(right_x * driftSpeed - turnPower);
            robot.driveRightFront.setPower(right_x * driftSpeed + turnPower);
        }else{

            robot.driveLeftFront.setPower(right_y - turnPower);
            robot.driveRightBack.setPower(right_y + turnPower);
            robot.driveLeftBack.setPower(right_y - turnPower);
            robot.driveRightFront.setPower(right_y + turnPower);

        }



    }

    private class reverseDrive extends Thread {
        reverseDrive() {
            this.setName("reverseDrive");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {
                    if (gamepad1.left_bumper) {
                        robot.reverse();
                        while (gamepad1.left_bumper && opModeIsActive()){
                            Thread.sleep(threadSleepTimeOut);

                        }

                    }
                    Thread.sleep(threadSleepTimeOut);
                }
            } catch (InterruptedException ignored) {
            }
        }
    }


    private class InputOperation extends Thread {
        InputOperation() {
            this.setName("InputOperation");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            try {
                while (!isInterrupted() && opModeIsActive()) {

                    if (gamepad1.left_trigger > 0.2) {
                        robot.setCollectMotorsPower(collectionSpeed * gamepad1.left_trigger);
                    }
                    if (gamepad1.right_trigger > 0.2) {
                        robot.setCollectMotorsPower(-collectionSpeed * gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger < 0.2) {
                        robot.setCollectMotorsPower(0);
                    }


                    if (gamepad1.right_bumper) {
                        collectionSpeed = 0.3;
                    } else {
                        collectionSpeed = 1;
                    }
                    Thread.sleep(threadSleepTimeOut);
                }
            } catch (InterruptedException ignored) {
            }
        }
    }

    private class catchers extends Thread {
        int position = 0;

        catchers() { setName("catchers"); }

        @Override
        public void run() {
            try {
                while (opModeIsActive() && !isInterrupted()) {
                    if (gamepad1.y) {
                        position = (position == 1) ? 0 : 1;
                        while (gamepad1.y && opModeIsActive()) {
                            Thread.sleep(threadSleepTimeOut);
                        }
                    }
                    robot.setCatchers(position);
                    Thread.sleep(threadSleepTimeOut);

                }

            } catch (InterruptedException ignored) {
            }
        }
    }
}





