package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;
import static org.firstinspires.ftc.teamcode.MathFunctions.inRange;
@TeleOp(name="testerBot", group="APOLLO")
@Disabled
public class TeleopTest extends functions {
    HardwareTester robot = new HardwareTester();
    public void runOpMode() {
        robot.init(hardwareMap, true);
        telemetry.addData("done", "init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("encoder:",robot.encoder.getCurrentPosition());
            telemetry.update();
        }
    }

}





