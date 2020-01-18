package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "Pure Pursuit")
public class Auto extends LinearOpMode {
    private RobotMovement robotMovement = new RobotMovement();
    ArrayList<CurvePoint> pathCurrent = new ArrayList();
    ArrayList<CurvePoint> path = new ArrayList();
    public void runOpMode(){
        robotMovement.robot.init(hardwareMap,false);
        path.add(new CurvePoint(40,12,1.0,0.5,50,Math.toRadians(90),1.0));
        path.add(new CurvePoint(40,22,1.0,0.5,50,Math.toRadians(90),1.0));
        path.add(new CurvePoint(20,12,1.0,0.5,50,Math.toRadians(90),1.0));

        path.add(new CurvePoint(20,80,1.0,0.5,50,Math.toRadians(180),1.0));
        path.add(new CurvePoint(40,80,1.0,0.5,50,Math.toRadians(180),1.0));

        telemetry.addData("finish","wating to start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("opMose","is active");
            telemetry.addData("bot point", new Point(robotMovement.botX(), robotMovement.botY()));
            telemetry.addData("pathCurrent.size",pathCurrent.size());

            telemetry.addData("error",robotMovement.error);
            telemetry.addData("steer",robotMovement.steer);
            telemetry.addData("pathCurrent.size()",pathCurrent.size());

            robotMovement.followCurve(path,1,0.4);
            telemetry.addData("followMe:", "X => " + robotMovement.followMe.x);

            telemetry.update();

        }
    }
}
