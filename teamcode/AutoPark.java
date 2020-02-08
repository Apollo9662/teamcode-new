package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;

@Autonomous(name = "Auto park", group = "Apollo")
public class AutoPark extends GyroOperator {
    public void runOpMode(){
        robot.init(hardwareMap, false);

        telemetry.addData("state ==> ", "Start");

        telemetry.update();
        waitForStart();
        sleep(25000);
        robot.backClaw.setPosition(backClosePos);
        robot.verticalElevator.setPower(1);
        sleep(950);
        robot.verticalElevator.setPower(-1);
        sleep(600);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(frontOpenPos);

        pidDrive(0.4,55,0,DriveMode.normal);
    }
}
