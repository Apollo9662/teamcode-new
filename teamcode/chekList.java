package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
@TeleOp(name = "Tester",group = "test")
public class chekList extends functions {
    double position;
    View relativeLayout;

    Hardware robot = new Hardware();
    public void runOpMode(){
        robot.init(hardwareMap,false);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
        telemetry.addData("hi","test");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            robot.setDriveMotorsPower(gamepad1.left_stick_y , Hardware.DRIVE_MOTOR_TYPES.ALL);
            robot.verticalElevator.setPower(gamepad2.right_stick_y);
            robot.horizontalElevator.setPower(gamepad2.left_stick_x);
            if (gamepad1.y) {
                position = (position == 1) ? 0 : 1;
                while (gamepad1.y) {
                }
            }
            robot.setCatchers(position);
            double backPosition;
            double frontPosition;

                if (gamepad2.left_trigger > 0.1) {
                    frontPosition = frontOpenPos;
                } else {
                    frontPosition = frontClosePos;
                }

                if (gamepad2.right_trigger > 0.1) {
                    backPosition = backOpenPos;
                } else {
                    backPosition = backClosePos;
                }
                if (gamepad1.left_trigger > 0.1) {
                    frontPosition = frontOpenPos;
                }

            robot.backClaw.setPosition(backPosition);
            robot.frontClaw.setPosition(frontPosition);
            robot.leftCollector.setPower(gamepad1.left_trigger);
            robot.rightCollector.setPower(gamepad1.left_trigger);
            telemetry.addData("front right", robot.driveRightFront.getCurrentPosition());
            telemetry.addData("back right", robot.driveRightBack.getCurrentPosition());
            telemetry.addData("front left", robot.driveLeftFront.getCurrentPosition());
            telemetry.addData("back left", robot.driveLeftBack.getCurrentPosition());
            telemetry.addData("horizontal", robot.horizontalElevator.getCurrentPosition());
            telemetry.update();

        }
    }
}
