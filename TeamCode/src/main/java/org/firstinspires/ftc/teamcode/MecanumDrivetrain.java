package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrivetrain {
    private Gamepad gamepad1;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private  DcMotorEx backRight;
    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        //retrieve motors from hardware map
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft"); //port 0
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright"); //port 1
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft"); //port 2
        backRight = hardwareMap.get(DcMotorEx.class, "backright"); //port 3

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setPower(IMU imu, double x, double y, double rx) {
        double rotX = x;
        double rotY = -y;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        flPower = (rotY - rotX + rx) / denominator;
        blPower = (rotY + rotX + rx) / denominator;
        frPower = (rotY - rotX - rx) / denominator;
        brPower = (rotY + rotX - rx) / denominator;
    }
    public void drive() {
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    //sets robot at halfspeed when a button is held
    public void slowDrive() {
        frontLeft.setPower(flPower / 2);
        frontRight.setPower(frPower / 2);
        backLeft.setPower(blPower / 2);
        backRight.setPower(brPower / 2);
    }
}
