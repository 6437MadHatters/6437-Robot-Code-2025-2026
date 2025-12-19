package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Methods {

    // config variable!!
    static Configuration config;

    //constructor for the method. Whenever an object is created, it needs to be passed the configuration object of the class that creates its object. This allows this class to use the objects of the motors/servos and make changes.
    public Methods(Configuration config) {
        Methods.config = config;
    }

    // VARIABLES

    // INTAKE VARIABLES
    //??

    // SHOOTER FLYWHEEL VARIABLES
    private final ElapsedTime pidTimer = new ElapsedTime(); // timer for motor PID

    // PID coefficients - TUNE THESE (watch a youtube tutorial or something)
    double kP = 0.05; // P
    double kI = 0.00015; // I
    double kD = 0.000075; // D
    double F = 0.00023; // feedforward coefficient! slope of best fit line - helps set to more than one speed
    double FFOffset = 0.113; // offset of the best-fit line of motor power to rpm
    double integral = 0;
    double lastError = 0;
    double lastTime = 0;
    int lastTarget = 0;
    double lastRPM = 0;
    private final ElapsedTime readyTimer = new ElapsedTime();
    boolean readyTimerRunning = false;
    int totError = 0;
    int loopNum = 0;
    int avgError = 0;

    // ANGLE CHANGE VARIABLES
    // ?

    // LAUNCH SERVO VARIABLES
    private final ElapsedTime leftDropTimer = new ElapsedTime(); // timers for servo state machines
    private final ElapsedTime midDropTimer = new ElapsedTime();
    private final ElapsedTime rightDropTimer = new ElapsedTime();

    enum launchState {
        idle,
        dropping,
        returning,
        finished
    }

    launchState leftState = launchState.idle;
    launchState midState = launchState.idle;
    launchState rightState = launchState.idle;
    boolean leftStarted = false;
    boolean midStarted = false;
    boolean rightStarted = false;
    boolean shotInProgress = false;
    double hold = 0;
    double drop = .15;
    double dropWait = .5;

    // ACTUAL METHODS

    // INTAKE CONTROL
    public void intakePower(double motorPower){
        config.intakeMotor.setPower(motorPower);
    }

    // SHOOTER CONTROL
    // sync shooter motors
    public void shooterPower(double motorPower) {
        config.shooterLeft.setPower(motorPower);
        config.shooterRight.setPower(motorPower);
    }

    // PID control to set a target motor RPM (returns true if shooter is up to speed)
    public boolean setRPM(int targetRPM, Telemetry telemetry) {

        // ACTUAL PID
        double currentTime = pidTimer.seconds();
        double timeDifference = currentTime - lastTime; // time change since start of last loop iteration
        if (timeDifference <= 0) {
            timeDifference = 0.001; // prevent division by 0
        }
        // rpm = motor ticks per second * 60 seconds / 28 encoder ticks per revolution
        double leftRPM = config.shooterLeft.getVelocity() * 60 / 28;
        double rightRPM = config.shooterRight.getVelocity() * 60 / 28;
        double currentRPM = (leftRPM + rightRPM) / 2;

        double error = targetRPM - currentRPM;
        integral += error * timeDifference;
        double derivative = (error - lastError) / timeDifference; // or this

        double feedForward = FFOffset + (F * targetRPM); // Use feedforward coefficient and target to get motor power
        double output = feedForward + (kP * error) + (kI * integral) + (kD * derivative); // and this is just magic.
        output = Math.max(0.01, Math.min(output, 1)); // make sure output is power between 0.01 and 1 (not 0 (zeroPower = motorBrake -> slight twitch))

        shooterPower(output); // set motor powers! (use output for pid, power for direct power)

        // reset integral when target changes (or goes below 500 (stopping))
        if ((targetRPM != lastTarget) || (lastRPM < 500)) {
            integral = 0;
        }
        lastTarget = targetRPM;
        lastRPM = getShooterSpeed();

        // update error and time
        lastError = error;
        lastTime = currentTime;

        // CHECK IF SHOOTER IS READY TO FIRE
        double readyTolerance = 150; // CHANGE THIS FOR MORE ACCURACY (how far from target RPM should the shooter be in order to shoot)
        double checkTime = .15; // shooter must be at target RPM for this long before launching

        // while the shooter is super close to its target speed, run a timer.
        if (Math.abs(error) < readyTolerance) {
            if (!readyTimerRunning) {
                readyTimer.reset();
                readyTimerRunning = true;
            }
        } else {
            readyTimerRunning = false;
        }
        // when the shooter has been at its target RPM for long enough, it's ready to shoot
        boolean ready = readyTimerRunning && (readyTimer.seconds() > checkTime);

        // add telemetry
        totError += (int) error;
        loopNum += 1;
        avgError = totError / loopNum;
        telemetry.addData("TargetRPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Error", error);
        telemetry.addData("avg. error", avgError);
        telemetry.addData("Pterm", kP * error);
        telemetry.addData("Iterm", kI * integral);
        telemetry.addData("Dterm", kD * derivative);
        telemetry.addData("Power", output);
        telemetry.addData("Ready to Shoot", ready);

        // return true if the shooter speed is close enough
        return ready;
    }

    // getter for shooter rpm
    public double getShooterSpeed() {
        double leftRPM = config.shooterLeft.getVelocity() * 60 / 28;
        double rightRPM = config.shooterRight.getVelocity() * 60 / 28;
        return (leftRPM + rightRPM) / 2;
    }

    // ANGLE CONTROL
    // convert angle to servo position
    public double angleToPos(double angle) {
        double clampedAngle;
            if (angle >= BotState.angleUp){
                clampedAngle = BotState.angleUp;
            } else if (angle <= BotState.angleDown) {
                clampedAngle = BotState.angleDown;
            } else {
                clampedAngle = angle;
            }
        double pos = 0.04467 * Math.pow((clampedAngle - 45), 0.7818); // magic numbers. don't ask just trust
        return Math.max(0, Math.min(pos, .7));
    }

    // set angle?? LIMELIGHT? nah

    // Update angle every loop in the runner ***SYNC THEIR SPEEDS (one is faster for some reason (only in one direction))
    public void updateAngle(double targetPose) {
        // set both to same speed
        config.angleLeft.setPosition(targetPose); // update angle servos
        config.angleRight.setPosition(targetPose);
    }

    // LAUNCH SERVO CONTROL
    // drop into shooter. state machine for servos
    public void shoot(boolean left, boolean mid, boolean right) {
        if (left && !leftStarted) {//leftState != launchState.dropping) {
            config.releaseLeft.setPosition(drop);
            leftState = launchState.dropping;
            leftDropTimer.reset();
            leftStarted = true;
        }
        if (mid && !midStarted) { //midState != launchState.dropping) {
            config.releaseMiddle.setPosition(drop);
            midState = launchState.dropping;
            midDropTimer.reset();
            midStarted = true;
        }
        if (right && !rightStarted) {//rightState != launchState.dropping) {
            config.releaseRight.setPosition(drop);
            rightState = launchState.dropping;
            rightDropTimer.reset();
            rightStarted = true;
        }
    }

    // Update launcher servo states in the runner
    public void updateServoState() {

        if (leftState == launchState.idle) {
            config.releaseLeft.setPosition(hold);
            leftStarted = false;
        } else if (leftState == launchState.dropping) {
            config.releaseLeft.setPosition(drop);
            if (leftDropTimer.seconds() > dropWait) {
                leftState = launchState.returning;
                leftDropTimer.reset();
            }
        } else if (leftState == launchState.returning) {
            config.releaseLeft.setPosition(hold);
            if (leftDropTimer.seconds() > dropWait) {
                leftState = launchState.idle;
            }
        }

        if (midState == launchState.idle) {
            config.releaseMiddle.setPosition(hold);
            midStarted = false;
        } else if (midState == launchState.dropping) {
            config.releaseMiddle.setPosition(drop);
            if (midDropTimer.seconds() > dropWait) {
                midState = launchState.returning;
                midDropTimer.reset();
            }
        } else if (midState == launchState.returning) {
            config.releaseMiddle.setPosition(hold);
            if (midDropTimer.seconds() > dropWait) {
                midState = launchState.idle;
            }
        }

        if (rightState == launchState.idle) {
            config.releaseRight.setPosition(hold);
            rightStarted = false;
        } else if (rightState == launchState.dropping) {
            config.releaseRight.setPosition(drop);
            if (rightDropTimer.seconds() > dropWait) {
                rightState = launchState.returning;
                rightDropTimer.reset();
            }
        } else if (rightState == launchState.returning) {
            config.releaseRight.setPosition(hold);
            if (rightDropTimer.seconds() > dropWait) {
                rightState = launchState.idle;
            }
        }

        shotInProgress = leftState != launchState.idle || midState != launchState.idle || rightState != launchState.idle;
    }

    // servo state getters
    public launchState getLeftState() {
        return leftState;
    }
    public launchState getMidState() {
        return midState;
    }
    public launchState getRightState() {
        return rightState;
    }
    public boolean isShotInProgress() {
        return shotInProgress;
    }
}