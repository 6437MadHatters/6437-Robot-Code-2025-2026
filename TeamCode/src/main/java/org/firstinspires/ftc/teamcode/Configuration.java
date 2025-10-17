package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

public class Configuration {
    //this class makes sense so you can have one set config. This is super useful if you want to change the direction of a motor or something.
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor shooterLeft;
    DcMotor shooterRight;
    Servo releaseLeft;
    Servo releaseMiddle;
    Servo releaseRight;

    public Configuration(HardwareMap hardwareMap) {
        //just define the motors and stuff
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        releaseLeft = hardwareMap.get(Servo.class, "releaseLeft");
        releaseMiddle = hardwareMap.get(Servo.class, "releaseMiddle");
        releaseRight = hardwareMap.get(Servo.class, "releaseRight");


        //I like to set the direction to forwards, even though it already does that, so that I can visualize what motors are doing what
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        releaseLeft.setDirection(Direction.REVERSE);
        releaseMiddle.setDirection(Direction.REVERSE);
        releaseRight.setDirection(Direction.FORWARD);

        //Brake or float, you should usually use brake. This means that the motor will not go limp, but instead try to resist movement.
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}