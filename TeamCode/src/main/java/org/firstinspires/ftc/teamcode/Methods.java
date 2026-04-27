package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class Methods {

    // config variable!!
    static Configuration config;

    //constructor for the method. Whenever an object is created, it needs to be passed the configuration object of the class that creates its object. This allows this class to use the objects of the motors/servos and make changes.
    public Methods(Configuration config) {
        Methods.config = config;
    }

    // VARIABLES

    // LIFT VARIABLES
    int topTicks = 675; // this is the top of the lift
    double liftUpPower = 1;
    double liftHoldPower = .1;

    // INTAKE VARIABLES
    ColorSensor colorSensor;
    DistanceSensor rangeSensor;
    double sensedDist = 0;
    enum alliance {
        blue,
        red,
        unknown
    }
    alliance currentAlliance = alliance.blue;
    int allianceID = 0;
    enum ballColor {
        green,
        purple,
        unknown,
        empty
    }
    int patternID = 0;
    ballColor[] currentPattern = {ballColor.unknown, ballColor.unknown, ballColor.unknown};
    ballColor currentColor = ballColor.empty;

    // SHOOTER FLYWHEEL VARIABLES
    private final ElapsedTime pidTimer = new ElapsedTime(); // timer for ALL PID's

    // PID coefficients - TUNE THESE (watch a youtube tutorial or something) (this one is a complicated PID)
    double kP = 0.005; // P
    double kI = 0.000005; // I
    double kD = 0.000075; // D
    double F = 0.00025; // feedforward! slope of best fit of power vs rpm - helps set to more than one speed - set slightly lower than true (.00027)
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
    int targetRPM = 0; // separate variable to get targetRPM in other classes

    // AUTO ALIGN VARIABLES
    double currentDepotTag;
    double currentTagArea = 0;
    double currentX = 0;
    double targetX = 0; // where do we want the center of the april tag
    double currentXAngle = 0;
    double finalTargetX = 0; // target after adjustment for X angle
    double Px = 0.025; // P variable for heading P controller. tune this
    double maxSpeedX = 1; // max speed of auto heading adjust
    double currentY = 0;
    double targetY = -9.5; // where do we want the center of the april tag
    double Py = 0.15; // P variable for angle P controller. tune this
    double maxSpeedY = 1.5; // max speed of auto angle adjust

    // LAUNCH SERVO VARIABLES
    private final ElapsedTime leftDropTimer = new ElapsedTime(); // timers for servo state machines
    private final ElapsedTime midDropTimer = new ElapsedTime();
    private final ElapsedTime rightDropTimer = new ElapsedTime();
    private final ElapsedTime scoopTimer = new ElapsedTime();

    enum servoState {
        idle, // not moving
        flicking, // middle drop/flick servo
        going, // dropping servos/moving to temporary position
        returning, // returning to original position
        finished // done moving
    }

    servoState leftState = servoState.idle;
    servoState midState = servoState.idle;
    servoState rightState = servoState.idle;
    servoState scoopState = servoState.idle;
    boolean flickStarted = false;
    boolean leftStarted = false;
    boolean midStarted = false;
    boolean rightStarted = false;
    boolean armsMoving = false;
    boolean scoopStarted = false;
    boolean scoopMoving = false;
    double out = .14;
    double hold = .26;
    double drop = .43;
    int scoopAngle = 60; // how high scoop method goes up; it was 45 :)
    double servoWait = .5;

    // ACTUAL METHODS

    // reset all PID errors
    public void pidErrorReset() {
        integral = 0;
    } // reset the total errors for all the PID's when the system closes. CALL THIS EVERY TIME YOU EXIT THE SHOOT CLASS

    // LIFT CONTROL
    public void updateLift(boolean goUp) {
        if (goUp) {
            if (config.liftMotor.getCurrentPosition() < topTicks) {
                config.liftMotor.setPower(liftUpPower); // power until the arm is past physically perpendicular with floor
            } else {
                config.liftMotor.setPower(liftHoldPower); // passed the middle, sink down to mechanical hard stop
            }
        }
        else {
            config.liftMotor.setPower(0);
        }
    } // CANNOT BE UNDONE: WHEN DPAD_DOWN IS PRESSED IN TELEOP THE BOT WILL LIFT AND STAY UP

    // INTAKE CONTROL
    // set intake power
    public void intakePower(double motorPower){
        config.intakeMotor.setPower(motorPower);
    }

    // PATTERN IDENTIFICATION
    // read the april tag ID for randomization
    public void readTagIDs(Telemetry telemetry, boolean isThisABackAuto){
        LLResult result = config.limelight.getLatestResult();
        if (result != null && result.isValid()){
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = (int) tag.getFiducialId();
                    if (id == 21 || id == 22 || id == 23) {
                        patternID = id;
                    } else if (id == 20 || id == 24) {
                        allianceID = id;
                    }
                }
                telemetry.addData("Detected Alliance", currentAlliance.toString() + " | " + allianceID);
                telemetry.addData("Detected Pattern", Arrays.toString(currentPattern) + " | " + patternID);
        } else {
            telemetry.addData("Limelight", "No targets seen");
        }
        updateAlliance(allianceID, isThisABackAuto);
        updatePattern(patternID);
    }

    // get april tag id's and set current alliance
    public void updateAlliance(int tagID, boolean isThisABackAuto){
        if (tagID == 24) {
            if (!isThisABackAuto) {
                currentAlliance = alliance.blue;
            } else {
                currentAlliance = alliance.red;
            }
        } else if (tagID == 20) {
            if (!isThisABackAuto) {
                currentAlliance = alliance.red;
            } else {
                currentAlliance = alliance.blue;
            }
        }
    }

    public alliance getCurrentAlliance(){
        return currentAlliance;
    }

    // get april tag id and set the current pattern
    public void updatePattern(int tagID){
        if (tagID == 21) {
            currentPattern = new ballColor[]{ballColor.green, ballColor.purple, ballColor.purple};
        } else if (tagID == 22) {
            currentPattern = new ballColor[]{ballColor.purple, ballColor.green, ballColor.purple};
        } else if (tagID == 23) {
            currentPattern = new ballColor[]{ballColor.purple, ballColor.purple, ballColor.green};
        } else {
            currentPattern = new ballColor[]{ballColor.unknown, ballColor.unknown, ballColor.unknown};
        }
    }

    // determine if a shooter has a ball and what color it is (call it every single loop somewhere)
    public ballColor getBallColor(ColorSensor colorSensor, DistanceSensor rangeSensor){
        // set up distance and color variables
        sensedDist = rangeSensor.getDistance(DistanceUnit.CM);
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        float[] hsvValues = {0F, 0F, 0F}; // idk why we're making a float list. apparently it makes the color sensor more accurate somehow
        Color.RGBToHSV(r, g, b, hsvValues);
        float hue = hsvValues[0];

        if (sensedDist > 4.9) { // yes, really, this is the exact sweet spot.
            currentColor = ballColor.empty;
        } else {
            if (hue >= 110 && hue <= 156) {
                currentColor = ballColor.green;
            } else if (hue >= 156 && hue <= 250) {
                currentColor = ballColor.purple;
            } else {
                currentColor = ballColor.unknown;
            }
        }

        return currentColor;
    }

    // sensor tests to check sensors and determine values
    // test distance
    public double getRange(DistanceSensor rangeTestSensor) {
        return rangeTestSensor.getDistance(DistanceUnit.CM);
    }

    // test color
    public float getHue(ColorSensor colorTestSensor){
        int r = colorTestSensor.red();
        int g = colorTestSensor.green();
        int b = colorTestSensor.blue();
        float[] hsvValues = {0F, 0F, 0F}; // idk why we're making a float list. apparently it makes the color sensor more accurate somehow
        Color.RGBToHSV(r, g, b, hsvValues);
        float hue = hsvValues[0];
        int[] list = {r, g, b};
        // return Arrays.toString(list);
        return hue;
    }

    // check if ANY shooter matches the pattern
    public boolean anyShooterMatches() {
        return currentPattern[0] == getBallColor(config.colorLeft, config.rangeLeft) || currentPattern[0] == getBallColor(config.colorMid, config.rangeMid)  || currentPattern[0] == getBallColor(config.colorRight, config.rangeRight);
    }

    // return true to shoot if the specific shooter should shoot
    public boolean shouldShoot(ColorSensor colorSensor, DistanceSensor rangeSensor){

        if (getBallColor(colorSensor, rangeSensor) == currentPattern[0]){
            return true; // if the ball matches the pattern, shoot
        } else if ((currentPattern[0] == ballColor.unknown) && (getBallColor(colorSensor, rangeSensor) != ballColor.empty)) {
            return true; // if no april tag is recognized just shoot one
        } else if (getBallColor(colorSensor, rangeSensor) != ballColor.empty && !anyShooterMatches()) {
            return true; // just dump if the shooter has a ball and nothing matches the pattern
        } else {
            return false;
        }
    }

    // rearrange the currentPattern after a shot is completed
    public void rearrangePattern() {
        ballColor currentColor = currentPattern[0];
        for (int i = 0; i < currentPattern.length - 1; i++) { // move every item left one space
            currentPattern[i] = currentPattern[i + 1];
        }
        currentPattern[currentPattern.length - 1] = currentColor; // place the color we just shot back on the end of the list
    }

    // SHOOTER CONTROL
    // sync shooter motors
    public void shooterPower(double motorPower) {
        config.shooterLeft.setPower(motorPower);
        config.shooterRight.setPower(motorPower);
    }

    // PID control to set a target motor RPM (returns true if shooter is up to speed)
    public boolean setRPM(int target, Telemetry telemetry) {

        targetRPM = target;

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
        double output = feedForward + (kP * error) + Math.max(0, Math.min((kI * integral), 0.075)) + (kD * derivative); // clamp I so it doesn't run away
        output = Math.max(0.01, Math.min(output, 1)); // make sure output is power between 0.01 and 1 (not 0 (zeroPower = motorBrake -> slight twitch))

        shooterPower(output); // set motor powers! (use output for pid, power for direct power)
        // shooterPower(1); // test powers

        lastTarget = targetRPM;
        lastRPM = getShooterSpeed();

        // update error and time
        lastError = error;
        lastTime = currentTime;

        // CHECK IF SHOOTER IS READY TO FIRE
        double readyTolerance = 150; // Tolerance should only really be 5% of the target
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
        //telemetry.addData("avg. error", avgError);
        //telemetry.addData("Pterm", kP * error);
        //telemetry.addData("Iterm", kI * integral);
        //telemetry.addData("Dterm", kD * derivative);
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
    // convert angle to servo position (clamp to 75 max and 45 min)
    public double angleToPos(double angle) {
        double clampedAngle;
            if (angle >= 75){
                clampedAngle = 75;
            } else if (angle <= 45) {
                clampedAngle = 45;
            } else {
                clampedAngle = angle;
            }
        double pos = (0.00000071943 * Math.pow(clampedAngle, 4)) - (0.000129484 * Math.pow(clampedAngle, 3)) + (0.00775377 * Math.pow(clampedAngle, 2)) - (0.146158 * clampedAngle) - 0.264893; // magic numbers. don't ask just trust
        return Math.max(0, Math.min(pos, .7));
    }

    // Update angle every loop in the runner THIS CALLS angleToPos!!! just send a raw angle to this method
    public void updateAngle(double targetAngle) {
        // set both to same speed
        config.angleLeft.setPosition(angleToPos(targetAngle)); // update angle servos
        config.angleRight.setPosition(angleToPos(targetAngle));
    }

    // AUTO FIRE ALIGNMENT
    // find values for what the limelight sees
    public void getLLDistances(Telemetry telemetry){
        config.limelight.pipelineSwitch(2); // pipeline 2 for fire alignment: only April tags !!! and !!!
        LLResult result = config.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            currentDepotTag = result.getFiducialResults().get(0).getFiducialId();
            //telemetry.addData("current depot tag ID", currentDepotTag);
            currentTagArea = result.getTa();
            telemetry.addData("April tag area", currentTagArea);
            currentX = result.getTx();
            //telemetry.addData("target x", currentX);
            currentXAngle = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getOrientation().getYaw();
            //telemetry.addData("X angle", currentXAngle);
            currentY = result.getTy();
            //telemetry.addData("target y", currentY);
        } else {
            telemetry.addData("Limelight", "No targets seen");
        }
    }

    // get best rpm based on april tag size
    public int alignRPM(boolean shootingThree, Telemetry telemetry){
        config.limelight.pipelineSwitch(2); // pipeline 2 for fire alignment: only April tags 20 and 24
        LLResult result = config.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double finalRPM;
            if (shootingThree) {
                finalRPM = 1.15 * 2483.30599 * Math.pow(currentTagArea, -0.175816); // higher rpm for 3 at once
            } else {
                finalRPM = 2483.30599 * Math.pow(currentTagArea, -0.175816); // fancy power regression to find ideal rpm
            }
            return (int) finalRPM;
        } else {
            return 2500; // if limelight can't see the april tag for some reason default to our most used shot
        }
    }

    // replace the left stick input with a P controller to keep the april tag centered
    public double autoLeftStickAlign(double manualInput, Telemetry telemetry){
        config.limelight.pipelineSwitch(2); // pipeline 2 for fire alignment: only April tags 20 and 24
        LLResult result = config.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // adjust target based on the angle of the april tag relative to the robot
            finalTargetX = targetX + (-0.001 * Math.pow(currentXAngle, 3) + (.32 * currentXAngle)); // adjust the target based on the angle. trust the magic.
            double errorX = currentX - finalTargetX;
            double adjustmentX; // add a fraction of the error every loop until there is no error. super simple. super smart.
            if (Math.abs(errorX) < 5){
                adjustmentX = Px * errorX * 2; // add more power when we get close to the target
            } else {
                adjustmentX = Px * errorX;
            }
            adjustmentX = Math.max(-maxSpeedX, Math.min(adjustmentX, maxSpeedX)); // Limit the maximum adjustment speed so it doesn't jerk too fast

            telemetry.addData("current left stick adjustment", adjustmentX);
            telemetry.addData("current X offset error", errorX);
            return adjustmentX;
        } else {
            return manualInput; // if limelight doesn't actually see anything return to manual control
        }
    }

    // align the shooter angle with the april tag - THIS IS A BASIC P CONTROLLER - only proportional
    public double alignShooterAngle(double angle, boolean shootingThree, Telemetry telemetry){
        config.limelight.pipelineSwitch(2); // pipeline 2 for fire alignment: only April tags 20 and 24
        LLResult result = config.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (shootingThree) {
                targetY = -9.75; // higher angle for 3 at once
            } else {
                targetY = -9.5;
            }
            double errorY = currentY - targetY;
            double adjustmentY = Py * errorY; // add a fraction of the error every loop until there is no error. super simple. super smart.
            adjustmentY = Math.max(-maxSpeedY, Math.min(adjustmentY, maxSpeedY)); // Limit the maximum adjustment speed so it doesn't jerk too fast
            angle += adjustmentY;

            telemetry.addData("current angle", angle);
            telemetry.addData("current Y offset error", errorY);
            //telemetry.addData("adjustment", adjustment);
            return Math.max(45, Math.min(angle, 75));
        } else {
            return angle; // if limelight doesn't actually see anything set to the last angle
        }
    }

    // LAUNCH SERVO CONTROL
    // drop into flywheel. state machine for servos
    public void shoot(boolean left, boolean mid, boolean right) {
        if (left && !leftStarted) {//leftState != launchState.dropping) {

            leftState = servoState.going;
            leftDropTimer.reset();
            leftStarted = true;
        }
        if (mid && !midStarted) { //midState != launchState.dropping) {
            config.releaseMiddle.setPosition(drop);
            midState = servoState.going;
            midDropTimer.reset();
            midStarted = true;
        }
        if (right && !rightStarted) {//rightState != launchState.dropping) {
            config.releaseRight.setPosition(drop);
            rightState = servoState.going;
            rightDropTimer.reset();
            rightStarted = true;
        }
    }

    public void flickOut(boolean flick) {
        if (flick && !flickStarted) { //midState != launchState.dropping) {
            config.releaseMiddle.setPosition(out);
            midState = servoState.flicking;
            midDropTimer.reset();
            flickStarted = true;
        }
    }

    // method to "scoop" - use state machine for drop servos and same logic as drop servos :)
    public void angleScoop (boolean triggered) {
        if (triggered && !scoopStarted) { //leftState != launchState.dropping) {
            updateAngle(scoopAngle);
            scoopState = servoState.going;
            scoopTimer.reset();
            scoopStarted = true;
        }
    }

    // Update launcher servo states AND the scoop state in the runner
    public void updateServoState() {

        if (leftState == servoState.idle) {
            config.releaseLeft.setPosition(hold);
            leftStarted = false;
        } else if (leftState == servoState.going) {
            config.releaseLeft.setPosition(drop);
            if (leftDropTimer.seconds() > servoWait) {
                leftState = servoState.returning;
                leftDropTimer.reset();
            }
        } else if (leftState == servoState.returning) {
            config.releaseLeft.setPosition(hold);
            if (leftDropTimer.seconds() > servoWait) {
                leftState = servoState.idle;
            }
        }

        if (midState == servoState.idle) {
            config.releaseMiddle.setPosition(hold);
            midStarted = false;
            flickStarted = false; // middle is a bit different because it can move backwards
        } else if (midState == servoState.going) {
            config.releaseMiddle.setPosition(drop);
            if (midDropTimer.seconds() > servoWait) {
                midState = servoState.returning;
                midDropTimer.reset();
            }
        } else if (midState == servoState.flicking) { // extra else-if for flick in the middle
            config.releaseMiddle.setPosition(out);
            if (midDropTimer.seconds() > servoWait) {
                midState = servoState.returning;
                midDropTimer.reset();
            }
        } else if (midState == servoState.returning) {
            config.releaseMiddle.setPosition(hold);
            if (midDropTimer.seconds() > servoWait) {
                midState = servoState.idle;
            }
        }

        if (rightState == servoState.idle) {
            config.releaseRight.setPosition(hold);
            rightStarted = false;
        } else if (rightState == servoState.going) {
            config.releaseRight.setPosition(drop);
            if (rightDropTimer.seconds() > servoWait) {
                rightState = servoState.returning;
                rightDropTimer.reset();
            }
        } else if (rightState == servoState.returning) {
            config.releaseRight.setPosition(hold);
            if (rightDropTimer.seconds() > servoWait) {
                rightState = servoState.idle;
            }
        }

        if (scoopState == servoState.idle) { // THIS ONE IS THE SCOOP STATE
            scoopStarted = false;
        } else if (scoopState == servoState.going) {
            updateAngle(scoopAngle);
            if (scoopTimer.seconds() > servoWait) {
                scoopState = servoState.returning;
                scoopTimer.reset();
            }
        } else if (scoopState == servoState.returning) {
            updateAngle(75);
            if (scoopTimer.seconds() > servoWait) {
                scoopState = servoState.idle;
            }
        }

        armsMoving = leftState != servoState.idle || midState != servoState.idle || rightState != servoState.idle;
        scoopMoving = scoopState != servoState.idle;
    }

    // servo state getters
    public servoState getLeftState() {
        return leftState;
    }
    public servoState getMidState() {
        return midState;
    }
    public servoState getRightState() {
        return rightState;
    }
    public servoState getScoopState() {
        return scoopState;
    }
    public boolean isArmMoving() {
        return armsMoving;
    }
    public boolean isScoopMoving(){
        return scoopMoving;
    }
}