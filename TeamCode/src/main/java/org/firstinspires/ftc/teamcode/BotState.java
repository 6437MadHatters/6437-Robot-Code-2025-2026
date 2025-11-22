package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotState {

    // config and methods variable!!
    static Configuration config;
    static Methods method;
    //constructor for the method. Whenever an object is created, it needs to be passed the configuration object of the class that creates its object. This allows this class to use the objects of the motors/servos and make changes.
    public BotState(Configuration config, Methods method) {
        BotState.config = config;
        BotState.method = method;
    }

    // VARIABLES
    //easy angles
    public static double angleUp = 75; // public static to use in other classes
    double angleMid = 60;
    public static double angleDown = 45; // public static to use in other classes
    double currentAngle;
    // ready to shoot
    boolean readyToShoot = false;
    // STATES
    enum botState {
        idle,
        shootOne,
        shootThree,
        intake,
        eject
    }
    botState currentState = botState.idle;

    // BOT STATE MACHINE
    // change state
    public void setBot(botState state){
        if (state == botState.idle){
            currentState = botState.idle;
        } else if (state == botState.shootOne) {
            currentState = botState.shootOne;
        } else if (state == botState.shootThree) {
            currentState = botState.shootThree;
        } else if (state == botState.intake) {
            currentState = botState.intake;
        } else if (state == botState.eject) {
            currentState = botState.eject;
        }
    }

    // update states every iteration of the loop IN THE RUNNER
    public void updateBotState(boolean A, boolean B, boolean X, boolean Y, Telemetry telemetry){
        if (currentState == botState.idle){
            method.intakePower(0);
            method.shooterPower(0);
            currentAngle = angleDown;
        } else if (currentState == botState.shootOne) {
            method.intakePower(0);
            readyToShoot = method.setRPM(2500, telemetry);
            currentAngle = angleMid;
            if (B && readyToShoot){
                method.shoot(false, false, true);
            } else if (X && readyToShoot) {
                method.shoot(true, false, false);
            } else if (Y && readyToShoot) {
                method.shoot(false, true, false);
            }
        } else if (currentState == botState.shootThree) {
            method.intakePower(0);
            readyToShoot = method.setRPM(3200, telemetry);
            currentAngle = angleMid;
            if (A && readyToShoot){
                method.shoot(true, true, true);
            }
        } else if (currentState == botState.intake) {
            method.intakePower(.9);
            method.shooterPower(0);
            currentAngle = angleUp;
        } else if (currentState == botState.eject) {
            method.intakePower(-.5);
            method.shooterPower(0);
            currentAngle = angleUp;
        }

        // telemetry and updates
        method.updateAngle(method.angleToPos(currentAngle)); // calls two methods: one to convert angle to pose, another to set servo pose
        telemetry.addData("angle pose", currentAngle);
        method.updateServoState();
        telemetry.addData("left state", method.getLeftState());
        telemetry.addData("middle state", method.getMidState());
        telemetry.addData("right state", method.getRightState());
    }

    // bot state getter
    public botState getBotState(){
        return currentState;
    }
}
