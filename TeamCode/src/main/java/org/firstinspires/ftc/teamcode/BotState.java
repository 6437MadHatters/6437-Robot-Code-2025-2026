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
    //easy angles
    public static double angleUp = 75; // public static to use in other classes
    double angleMid = 60;
    public static double angleDown = 45; // public static to use in other classes
    double currentAngle;
    // ready to shoot
    boolean readyToShoot = false; // check if the flywheel is up to speed

    // STATES
    enum botState {
        idle,
        slowSpeed,
        fastSpeed,
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

    // BOT STATE MACHINE
    // change state
    public void setBot(botState state) {
        currentState = state;
    }

    // update states every iteration of the loop IN THE RUNNER
    public void updateBotState(boolean allA, boolean rightB, boolean leftX, boolean middleY) {

        if (currentState == botState.idle) {
            method.intakePower(0);
            method.shooterPower(0);
            currentAngle = angleDown;
        } else if (currentState == botState.intake) {
            method.intakePower(.9);
            method.shooterPower(0);
            currentAngle = angleUp;

            // flick
            method.flickOut(allA);

        } else if (currentState == botState.eject) {
            method.intakePower(-.5);
            method.shooterPower(0);
            currentAngle = angleUp;
        } else if (currentState == botState.slowSpeed) {
            method.intakePower(0);
            readyToShoot = method.setRPM(2500, telemetry);
            currentAngle = angleMid;

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
        } else if (currentState == botState.fastSpeed) {
            method.intakePower(0);
            readyToShoot = method.setRPM(3200, telemetry);
            currentAngle = 50;

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
        method.updateAngle(method.angleToPos(currentAngle)); // calls two methods: one to convert angle to pose, another to set servo pose
        method.updateServoState(); // CHANGES DROPDONE
        lastState = currentState;


        // telemetry
        telemetry.addData("angle pose", currentAngle);
        telemetry.addData("middle drop state", method.getMidState());
        telemetry.addData("right drop state", method.getRightState());
        telemetry.addData("left drop state", method.getLeftState());
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
            boolean shotStarted = false; // boolean to check if servo fsm has started
            boolean latched = false; // latch shots if using pattern
            boolean shootRight, shootLeft, shootMid; // shoot booleans for patterns only

            public boolean run(@NonNull TelemetryPacket packet) {

                rrSetState = state;

                boolean needsToShoot = usePattern || allA || rightB || leftX || middleY; // check if we even requested to shoot.

                if (usePattern) {
                    if (!latched) { // latch pattern shots
                        shootRight = method.shouldShoot("right");
                        shootLeft  = method.shouldShoot("left");
                        shootMid   = method.shouldShoot("mid");
                        latched = true;
                    }
                    rrAllA = false;
                    rrRightB = shootRight;
                    rrLeftX  = shootLeft;
                    rrMiddleY = shootMid;
                } else {
                    rrAllA = allA;
                    rrRightB = rightB;
                    rrLeftX = leftX;
                    rrMiddleY = middleY;
                }

                // stop and wait for shooter to spin up if we're in position to shoot
                if (state == botState.slowSpeed || state == botState.fastSpeed) {

                    if (!needsToShoot || (!rrAllA && !rrRightB && !rrLeftX && !rrMiddleY)) {
                        return false; // stop if there's nothing to shoot
                    }

                    if (!readyToShoot) {
                        return true; // wait for flywheel
                    }

                     if (method.isShotInProgress()) {
                        shotStarted = true; // shot is currently happening
                        return true;
                    }

                    if (shotStarted && !method.isShotInProgress()) {
                        rrAllA = rrRightB = rrLeftX = rrMiddleY = false; // shot finished
                        method.rearrangePattern();
                        return false;
                    }

                    return true; // ready, but shot hasn’t started yet
                }

                return false; // not in a shooting state
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
                updateBotState(rrAllA, rrRightB, rrLeftX, rrMiddleY); // call updateBotState every loop to keep PID running and shoot artifacts
                return true;
            }
        };
    }
}