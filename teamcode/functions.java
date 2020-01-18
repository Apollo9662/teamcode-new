package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class functions extends RobotMovement {
    Hardware robot = new Hardware();
    double distanceToTarget;
    double encoder = 0;
    double distance = 0;

    @Override
    public void runOpMode()  {
    }

    public void driveToPosition(double targetX, double targetY){
        distanceToTarget = Math.sqrt(Math.pow(targetX - robot.point().x,2) + Math.pow(targetY - robot.point().y,2));
        while(distanceToTarget > 20 && opModeIsActive()){
            distanceToTarget = Math.sqrt(Math.pow(targetX - robot.point().x,2) + Math.pow(targetY - robot.point().y,2));
            robot.setDriveMotorsPower(0.2, Hardware.DRIVE_MOTOR_TYPES.ALL);
            distance = robot.driveRightBack.getCurrentPosition() - encoder;
            robot.point().x += distance*Math.cos(Math.toRadians(robot.GetGyroAngle()));
            robot.point().y += distance*Math.sin(Math.toRadians(robot.GetGyroAngle()));
            telemetry.addData("state","driving");
            telemetry.addData("robot.position.x",robot.point().x);
            telemetry.addData("robot.position.y",robot.point().y);
            telemetry.addData("dic",distance);
            telemetry.addData("dic to target",distanceToTarget);
            telemetry.addData("encoer",robot.driveRightBack.getCurrentPosition());
            telemetry.update();
            encoder = robot.driveRightBack.getCurrentPosition();
        }
    }
}


