package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotState {

    // config and methods variable!!
    static Configuration config;
    static Methods method;
    static Telemetry telemetry;

    //constructor for the method. Whenever an object is created, it needs to be passed the configuration object of the class that creates its object. This allows this class to use the objects of the motors/servos and make changes.
    public BotState(Configuration config, Methods method, Telemetry telemetry) {
        BotState.config = config;
        BotState.method = method;
        BotState.telemetry = telemetry;
    }

    // VARIABLES
    // REMEMBER THAT ANGLES RANGE FROM 45 TO 75!
    double currentAngle;
    // ready to shoot
    boolean readyToShoot = false; // check if the flywheel is up to speed

    // STATES
    enum botState {
        idle,
        shoot,
        intake,
        eject
    }

    botState currentState = botState.idle;
    botState lastState = null;

    // Action variables (USE THESE IN AUTO)
    botState rrSetState = botState.idle;
    boolean rrAllA = false;
    boolean rrLeftX = false;
    boolean rrRightB = false;
    boolean rrMiddleY = false;
    int rrRPM = 0;
    double rrAngle = 60;

    // BOT STATE MACHINE
    // change state
    public void setBot(botState state) {
        currentState = state;
    }

    // update states every iteration of the loop IN THE RUNNER
    public void updateBotState(boolean allA, boolean rightB, boolean leftX, boolean middleY, int rpm, double angle) {

        if (currentState == botState.idle) {
            method.intakePower(0);
            method.shooterPower(0);
            currentAngle = 45;
        } else if (currentState == botState.intake) {
            method.intakePower(1); // INTAKE POWER IS HERE
            method.shooterPower(0);
            currentAngle = 75; // needs to be all the way up when intaking
            method.flickOut(allA); // flick if allA is "pressed"
            method.angleScoop(rightB); // "scoop" if rightB is "pressed"

        } else if (currentState == botState.eject) {
            method.intakePower(-.5);
            method.shooterPower(0);
            currentAngle = 75; // and ejecting I guess
        } else if (currentState == botState.shoot && rpm != 0) {
            method.intakePower(0);
            readyToShoot = method.setRPM(rpm, telemetry);
            currentAngle = angle;

            // shoot when ready
            if (readyToShoot) {
                if (allA) {
                    method.shoot(true, true, true);
                } else if (rightB) {
                    method.shoot(false, false, true);
                } else if (leftX) {
                    method.shoot(true, false, false);
                } else if (middleY) {
                    method.shoot(false, true, false);
                }
            }
        }

        // updates
        method.updateAngle(currentAngle); // calls two methods: one to convert angle to pose, another to set servo pose
        method.updateServoState(); // CHANGES DROPDONE
        lastState = currentState;


        // telemetry
        telemetry.addData("angle pose", currentAngle);
        telemetry.addData("middle drop state", method.getMidState());
        telemetry.addData("right drop state", method.getRightState());
        telemetry.addData("left drop state", method.getLeftState());
        telemetry.addData("scoop state", method.getScoopState());
        telemetry.addData("left color", method.getBallColor("left"));
        telemetry.addData("middle color", method.getBallColor("mid"));
        telemetry.addData("right color", method.getBallColor("right"));
        // telemetry.addData("dist", method.getSensedDist()); // distance test
        // telemetry.addData("hue", method.getHue()); // hue test
    }

    // bot state getter
    public botState getBotState() {
        return currentState;
    }

    // RR ACTION TO CHANGE STATE INSTANTANEOUSLY (until a drop completes (or doesn't even start))
    public Action setBotAction (botState state, boolean usePattern, boolean allA, boolean rightB, boolean leftX, boolean middleY) {
        return new Action() {
            boolean moveStarted = false; // boolean to check if servo fsm has started
            boolean latched = false; // latch shots if using pattern
            boolean shootRight, shootLeft, shootMid; // shoot booleans for patterns only

            public boolean run(@NonNull TelemetryPacket packet) {

                rrSetState = state;

                boolean needsToAct = usePattern || allA || rightB || leftX || middleY; // check if we even requested to move

                // define rrVariables based on argument
                if (usePattern) {
                    if (!latched) { // latch pattern shots so only one can go at a time
                        shootRight = method.shouldShoot("right");
                        shootLeft  = method.shouldShoot("left");
                        shootMid   = method.shouldShoot("mid");
                        latched = true;
                    }
                    rrAllA = false;
                    rrRightB = shootRight;
                    rrLeftX  = shootLeft;
                    rrMiddleY = shootMid;
                    rrRPM = 2050;
                    rrAngle = 65;
                } else {
                    rrAllA = allA;
                    rrRightB = rightB;
                    rrLeftX = leftX;
                    rrMiddleY = middleY;
                    rrRPM = 2500;
                    rrAngle = 65;
                }

                // SHOOT STATE
                if (state == botState.shoot) {

                    if (!needsToAct || (!rrAllA && !rrRightB && !rrLeftX && !rrMiddleY)) {
                        return false; // stop if there's nothing to shoot
                    }

                    if (!readyToShoot) {
                        return true; // wait for flywheel
                    }

                     if (method.isArmMoving()) {
                        moveStarted = true; // shot is currently happening
                        return true;
                    }

                    if (moveStarted && !method.isArmMoving()) {
                        rrAllA = rrRightB = rrLeftX = rrMiddleY = false; // shot finished
                        method.rearrangePattern();
                        return false;
                    }

                    return true; // ready, but shot hasn’t started yet
                }

                // INTAKE STATE
                if (state == botState.intake) {

                    // allA = flick
                    // rightB = scoop

                    if (!needsToAct || (!rrAllA && !rrRightB)) {
                        return false; // no move requested
                    }

                    if (method.isArmMoving() || method.isScoopMoving()) {
                        moveStarted = true; // flick OR scoop has started moving
                        return true;
                    }

                    if (!moveStarted){
                        return true; // wait for movement to start
                    }

                    if (!method.isArmMoving() && !method.isScoopMoving()) {
                        rrAllA = rrRightB = rrLeftX = rrMiddleY = false; // movement finished
                        return false;
                    }

                    // don't return true, we don't need to wait for anything
                }

                return false; // not in shooting or intake state
            }
        };
    }

    // RR ACTION TO UPDATE STATE CONTINUOUSLY
    public Action updateStateAction() {

        return new Action() {

            public boolean run(@NonNull TelemetryPacket packet) {
                if (currentState != rrSetState) {
                    setBot(rrSetState); // call setBot and set the bot state once when the action is called
                }
                updateBotState(rrAllA, rrRightB, rrLeftX, rrMiddleY, rrRPM, rrAngle); // call updateBotState every loop to keep PID running and shoot artifacts
                return true;
            }
        };
    }
}