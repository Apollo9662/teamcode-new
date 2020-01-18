package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class HardwareTester {
    DcMotor encoder = null;
    private double s4tPPR = 3200;
    private double s4tGear = 3;
    private double s4tWheelDiameter = 2;

    private double s4tToIn = (s4tPPR * s4tGear) / (s4tWheelDiameter * Math.PI);
    public HardwareTester(){

    }
    void init(HardwareMap ahwMap,boolean teleop) {
        encoder = ahwMap.get(DcMotor.class, "encoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
 }


