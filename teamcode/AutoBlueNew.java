package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.left;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.normal;
import static org.firstinspires.ftc.teamcode.GyroOperator.DriveMode.right;
import static org.firstinspires.ftc.teamcode.Hardware.backClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.backOpenPos;
import static org.firstinspires.ftc.teamcode.Hardware.frontClosePos;
import static org.firstinspires.ftc.teamcode.Hardware.frontOpenPos;

@Autonomous(name = "Autonomous blue", group = "apollo")
public  class AutoBlueNew extends autonum{
    private Thread outPutCube = new outPutCube();
    private Thread timer = new timer();
    private ElapsedTime runtime = new ElapsedTime();
    skyStoneVision.stonePosition vu;
    enum stonePositionNewMoves{
        LEFT,
        CENTER,
        RIGHT
    }
    private InternalCameraExample openCV  = new InternalCameraExample();
    public void runOpMode(){
        robot.init(hardwareMap, false);

        telemetry.addData("imu is working ==> ", robot.imuIsWorking);

        telemetry.update();

        openCV.initOpenCV(hardwareMap);

        openCV.proces(0);
        waitForStart();

        if(!robot.imuIsWorking){
            while (opModeIsActive()){}
        }
        runtime.reset();
        timer.start();
        vu =  openCV.vision.position;

        robot.backClaw.setPosition(backClosePos);
        robot.verticalElevator.setPower(1);
        sleep(950);
        robot.verticalElevator.setPower(-1);
        sleep(600);
        robot.verticalElevator.setPower(0);
        robot.frontClaw.setPosition(frontOpenPos);
        pidDrive(0.35,38,0,normal);
        robot.setCollectMotorsPower(1);
        gyroTurn(0.4,-90);
        switch (vu){
            case right:
                newLeft();
                break;
            case center:
                newCenter();
                break;
            case left:
                newRight();
                break;
        }

        gyroTurn(0.25,-85);
        pidDrive(0.4,-20,-90,normal);
        robot.setCatchers(0);
        sleep(800);
        outPutCube.start();
//        robot.horizontalElevator.setPower(1);
//        sleep(700);
//        robot.horizontalElevator.setPower(0);
        pidDrive(1,20,-63,normal);
        pidDrive(1,20,-43,normal);
        pidDrive(1,45,0 , normal);
        gyroTurn(0.5,0);
        robot.setCatchers(1);
        pidDrive(0.7,-18,0,normal);
        pidDrive(0.7,70,0,normal);
        if (30 - runtime.seconds() >= 6){
//            seconedPart();
        }

    }

    public void newRight(){
        pidDrive(0.4,-8,-90,normal);
        pidDrive(0.4,55,-50,normal);
        robot.setCatchers(1);
        sleep(800);
        robot.frontClaw.setPosition(frontClosePos);
        pidDrive(0.4,-20,-50,normal);
        pidDrive(0.4,-50,-85,normal);
        robot.setCollectMotorsPower(0);
        pidDrive(0.8,-115,-85,normal);
        robot.initImu();
    }
    public void newCenter(){
        pidDrive(0.4,55,-50,normal);
        robot.setCatchers(1);
        sleep(800);
        robot.frontClaw.setPosition(frontClosePos);
        pidDrive(0.4,-20,-50,normal);
        pidDrive(0.4,-50,-85,normal);
        robot.setCollectMotorsPower(0);
        pidDrive(0.8,-113,-85,normal);
        robot.initImu();
    }
    public void newLeft(){
        pidDrive(0.4,18,-90,normal);
        pidDrive(0.4,55,-50,normal);
        robot.setCatchers(1);
        sleep(800);
        robot.frontClaw.setPosition(frontClosePos);
        pidDrive(0.4,-20,-50,normal);
        pidDrive(0.4,-50,-85,normal);
        robot.setCollectMotorsPower(0);
        pidDrive(0.8,-135,-85,normal);
        robot.initImu();
    }
    public void seconedPart(){
        robot.initImu();
        robot.setCollectMotorsPower(1);
        pidDrive(0.8, 65 , -15 ,normal);
        sleep(600);
        pidDrive(0.8,-65,0, normal);
        outPutCube.start();
        pidDrive(0.8,-100,0,normal);
        sleep(300);
        pidDrive(0.9,80,0,normal);
    }
    private class outPutCube extends Thread {
        int position = 0;

        outPutCube() { setName("catchers"); }

        @Override
        public void run() {
            try {
                robot.backClaw.setPosition(backOpenPos);
                robot.frontClaw.setPosition(frontOpenPos);
                sleep(150);
                robot.backClaw.setPosition(backClosePos);
                robot.frontClaw.setPosition(frontClosePos);
                robot.horizontalElevator.setPower(-1);
                robot.verticalElevator.setPower(0.2);
                sleep(1800);
                robot.verticalElevator.setPower(0);
                robot.horizontalElevator.setPower(0);
                robot.backClaw.setPosition(backOpenPos);
                robot.frontClaw.setPosition(frontOpenPos);
                sleep(700);
                robot.horizontalElevator.setPower(1);
                sleep(1800);
                robot.horizontalElevator.setPower(0);
                sleep(1000);
                robot.backClaw.setPosition(backClosePos);
                Thread.sleep(50);

            } catch (InterruptedException ignored) {
            }
        }
    }

    private class timer extends Thread {
        int position = 0;

        timer() { setName("timer"); }

        @Override
        public void run() {
            try {
                while (opModeIsActive() && !isInterrupted()) {
                    telemetry.addData("time", runtime.seconds());
                    telemetry.addData("pos  - ",vu);
                    telemetry.addData("gyro  - ",robot.GetGyroAngle());
                    telemetry.update();
                    sleep(50);
                }

            } catch (InterruptedException ignored) {
            }
        }
    }
}
