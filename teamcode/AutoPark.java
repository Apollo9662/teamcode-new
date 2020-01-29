package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto park", group = "Apollo")
public class AutoPark extends GyroOperator {
    public void runOpMode(){
        Hardware robot = new Hardware();
        robot.init(hardwareMap, false);
        waitForStart();
        pidDrive(0.4,10,0,DriveMode.normal);
    }
}
