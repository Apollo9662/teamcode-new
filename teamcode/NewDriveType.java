package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turn mid Drive")   // tipshim
public class NewDriveType extends functions {
    Hardware robot = new Hardware();
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    public void runOpMode(){
        robot.init(hardwareMap,false);
        telemetry.addData("gyro",robot.GetGyroAngle());
        telemetry.update();
        waitForStart();
        if (opModeIsActive())  Test(0.8,0,0.8,45);
    }
    public void Test(double speed, double angle, double turnSpeed,double turnAngle){
        double error;
        double steer;
        while (opModeIsActive()) {
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);
            steer = MathFunctions.map(steer,-1,1,-turnSpeed,turnSpeed);

            angle = angle + robot.GetGyroAngle() + 45;
            angle = MathFunctions.AngleWrap(Math.toRadians(angle));
            double x = speed * Math.cos(angle);
            double y = speed * Math.sin(angle);
            robot.driveRightFront.setPower(y - steer);
            robot.driveRightBack.setPower(x - steer);
            robot.driveLeftBack.setPower(y + steer);
            robot.driveLeftFront.setPower(x + steer);
        }

    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetGyroAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
