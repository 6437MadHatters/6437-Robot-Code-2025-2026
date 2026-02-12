package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.BotState.botState;


@TeleOp(name = "teleOp", group = "Linear OpMode")
public class teleOp extends LinearOpMode {

    // Declare OpMode variables.
    // time
    private final ElapsedTime runtime = new ElapsedTime();

    // these variables are for headless mode, I don't really understand it, I stole them from 3189
    double driveTurn;
    double driveVertical;
    double driveHorizontal;
    double gamepadX;
    double gamepadY;
    double gamepadHypot;
    double gamepadDegree;
    double robotDegree;
    double movementDegree;
    double gamepadXControl;
    double gamepadYControl;

    // whether or not headless mode is on
    boolean headlessMode = false;

    // controller trigger states -_- these are annoying af
    boolean lastTrigger = false;
    // method for button ...
    boolean triggerTap(double triggerValue) {
        boolean isPressed = triggerValue > 0.5;
        boolean tapped = isPressed && !lastTrigger;
        lastTrigger = isPressed;
        return tapped;
    }

    // lift state... this is also annoying
    boolean goUp = false;

    // I HATE THIS
    enum shootMode {
        far,
        close,
        none
    }
    shootMode currentShootMode = shootMode.none;

    int rpm = 0;
    double angle = 60;

    // class variables.
    Configuration config;
    Methods method;
    BotState state;

    // START RUNNING !!! \/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize ALL other classes with the proper class argument. !!!!! IMPORTANT
        config = new Configuration(hardwareMap);
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // variable to track what state we're in
        botState lastState = botState.idle;

        // define all the button objects that will use the button class
        // gamepad 1
        Button gamepad1A = new Button(gamepad1.a);
        Button gamepad1B = new Button(gamepad1.b);
        Button gamepad1X = new Button(gamepad1.x);
        Button gamepad1Y = new Button(gamepad1.y);
        Button gamepad1Dpad_Up = new Button(gamepad1.dpad_up);
        Button gamepad1Dpad_Down = new Button(gamepad1.dpad_down);
        Button gamepad1Dpad_Left = new Button(gamepad1.dpad_left);
        Button gamepad1Dpad_Right = new Button(gamepad1.dpad_right);
        Button gamepad1_right_stick_button = new Button(gamepad1.right_stick_button);
        Button gamepad1_left_stick_button = new Button(gamepad1.left_stick_button);
        Button gamepad1_Right_bumper = new Button(gamepad1.right_bumper);
        Button gamepad1_Left_bumper = new Button(gamepad1.left_bumper);
        // gamepad 2
        Button gamepad2A = new Button(gamepad2.a);
        Button gamepad2B = new Button(gamepad2.b);
        Button gamepad2X = new Button(gamepad2.x);
        Button gamepad2Y = new Button(gamepad2.y);
        Button gamepad2Dpad_Up = new Button(gamepad2.dpad_up);
        Button gamepad2Dpad_Down = new Button(gamepad2.dpad_down);
        Button gamepad2Dpad_Left = new Button(gamepad2.dpad_left);
        Button gamepad2Dpad_Right = new Button(gamepad2.dpad_right);
        Button gamepad2_right_stick_button = new Button(gamepad2.right_stick_button);
        Button gamepad2_left_stick_button = new Button(gamepad2.left_stick_button);
        Button gamepad2_Right_bumper = new Button(gamepad2.right_bumper);
        Button gamepad2_Left_bumper = new Button(gamepad2.left_bumper);

        // initialize the gyro (we only use it for driving)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // INIT POSITION
        config.liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        config.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        method.updateLift(goUp);

        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false, rpm, angle);

        // tell when init is done
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();
        // START THE TIMER WHEN ROBOT STARTS
        runtime.reset();
        long savedTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //update all of the buttons
            // gamepad 1
            gamepad1A.update(gamepad1.a);
            gamepad1B.update(gamepad1.b);
            gamepad1X.update(gamepad1.x);
            gamepad1Y.update(gamepad1.y);
            gamepad1Dpad_Up.update(gamepad1.dpad_up);
            gamepad1Dpad_Down.update(gamepad1.dpad_down);
            gamepad1Dpad_Left.update(gamepad1.dpad_left);
            gamepad1Dpad_Right.update(gamepad1.dpad_right);
            gamepad1_right_stick_button.update(gamepad1.right_stick_button);
            gamepad1_left_stick_button.update(gamepad1.left_stick_button);
            gamepad1_Right_bumper.update(gamepad1.right_bumper);
            gamepad1_Left_bumper.update(gamepad1.left_bumper);
            // gamepad 2
            gamepad2A.update(gamepad2.a);
            gamepad2B.update(gamepad2.b);
            gamepad2X.update(gamepad2.x);
            gamepad2Y.update(gamepad2.y);
            gamepad2Dpad_Up.update(gamepad2.dpad_up);
            gamepad2Dpad_Down.update(gamepad2.dpad_down);
            gamepad2Dpad_Left.update(gamepad2.dpad_left);
            gamepad2Dpad_Right.update(gamepad2.dpad_right);
            gamepad2_right_stick_button.update(gamepad2.right_stick_button);
            gamepad2_left_stick_button.update(gamepad2.left_stick_button);
            gamepad2_Right_bumper.update(gamepad2.right_bumper);
            gamepad2_Left_bumper.update(gamepad2.left_bumper);

            // BOT STATE MACHINE (change states!!!)
            // shoot close mode
            if (triggerTap(gamepad1.right_trigger)) {
                if (currentShootMode != shootMode.close) {
                    state.setBot(botState.shoot);
                    lastState = botState.shoot;
                    rpm = 2500;
                    angle = 60;
                    currentShootMode = shootMode.close;
                } else {
                    state.setBot(botState.idle);
                    lastState = botState.idle;
                }
            }

            // shoot far mode
            if (gamepad1_Right_bumper.buttonPress()) {
                if (currentShootMode != shootMode.far) {
                    state.setBot(botState.shoot);
                    lastState = botState.shoot;
                    rpm = 3200;
                    angle = 50;
                    currentShootMode = shootMode.far;
                } else {
                    state.setBot(botState.idle);
                    lastState = botState.idle;
                }
            }

             // Intake
             if (gamepad1_Left_bumper.buttonPress()){
                if (state.getBotState() != botState.intake) {
                    state.setBot(botState.intake);
                    lastState = botState.intake;
                } else {
                    state.setBot(botState.idle);
                    lastState = botState.idle;
                }
            }

             // Eject
             if (gamepad1.left_trigger > .5) {
                 state.setBot(botState.eject);
             } else if (state.getBotState() == botState.eject) {
                 state.setBot(botState.idle);
             }

             // Lift
            if (gamepad1Dpad_Up.buttonPress()) {
                config.liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                config.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                state.setBot(botState.idle);
                goUp = true;
            }
            method.updateLift(goUp);

            // UPDATE BOT STATE EVERY ITERATION OF THE LOOP (arguments are for buttons that are used in updateBotState)
            state.updateBotState(
                    (gamepad1A.buttonPress()),
                    (gamepad1B.buttonPress()),
                    (gamepad1X.buttonPress()),
                    (gamepad1Y.buttonPress()),
                    rpm,
                    angle
            );

            if (state.getBotState() != botState.shoot) {
                currentShootMode = shootMode.none; // set the currentShootMode to none if we aren't shooting.
            }

            telemetry.addData("Current State", state.getBotState());
            telemetry.addData("lift position", config.liftMotor.getCurrentPosition());
            telemetry.addData("lift power", config.liftMotor.getPower());


            // DRIVE !!! \/

            // headless mode toggle (right stick toggle headless, left stick reset gyro)
            /* if (gamepad1_right_stick_button.buttonPress()) {
                headlessMode = !headlessMode;
            } */
            telemetry.addData("Headless Mode", headlessMode);

            // SPEED
            double turnMultiplier = .85;

            // telemetry.addData("turn multiplier (drive speed): ", turnMultiplier);

            //drive code, I don't really understand much.
            if (headlessMode) {
                driveTurn = -gamepad1.left_stick_x;
                driveHorizontal = gamepad1.right_stick_x;
                driveVertical = -gamepad1.right_stick_y;

                gamepadX = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
                gamepadY = -gamepad1.right_stick_y; //this simply gives our y value relative to the driver
                gamepadHypot = Range.clip(Math.hypot(gamepadX, gamepadY), 0, 1);
                // finds just how much power to give the robot based on how much x and y given by gamepad
                // range.clip helps us keep our power within positive 1
                // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)

                // prevent /0 error
                if (gamepadX != 0) {
                    //calculates gamepad degree with inverse tangent
                    gamepadDegree = Math.toDegrees(Math.atan(gamepadY / gamepadX));
                    //if stick is left, make angle left
                    if (gamepadX < 0) {
                        gamepadDegree += 180;
                    }
                }
                //straight up
                else if (gamepadY > 0) {
                    gamepadDegree = 90;
                }
                // straight down
                else if (gamepadY < 0) {
                    gamepadDegree = 270;
                }
                // gamepad has not been moved
                else {
                    gamepadDegree = 0;
                }

                // RESET GYRO IF NEEDED !!!!!!!!!!!!
                if (gamepad1.left_stick_button) {
                    imu.initialize(parameters);
                }

                // telemetry.addData("runtime - saved time", runtime.now(TimeUnit.MILLISECONDS) - savedTime);

                //ANGLE MATH
                //gives us the angle our robot is at
                robotDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
                // telemetry.addData("Robot degree", robotDegree);
                //find angle relative to robot position
                movementDegree = gamepadDegree - robotDegree;
                //by finding the adjacent side, we can find sideways movement relative to robot
                gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
                driveHorizontal = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the opposite side, we can find vertical movement relative to robot
                gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
                driveVertical = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;


                //drive math
                double strafeSpeed = 1.5;
                driveTurn *= .7;
                // yaw speed
                config.frontRight.setPower((-driveVertical + (driveHorizontal * strafeSpeed) + driveTurn) * turnMultiplier);
                config.backRight.setPower((-driveVertical - (driveHorizontal * strafeSpeed) + driveTurn) * turnMultiplier);
                config.frontLeft.setPower((-driveVertical - (driveHorizontal * strafeSpeed) - driveTurn) * turnMultiplier);
                config.backLeft.setPower((-driveVertical + (driveHorizontal * strafeSpeed) - driveTurn) * turnMultiplier);
            } else {
                //basic mecanum code (MAKE STICK INPUTS NEGATIVE TO CHANGE ROBOT DIRECTION (RIGHT STICK ONLY!!!!)) !!!!!!!!!!!!!!!!
                config.frontLeft.setPower((-gamepad1.right_stick_y + (gamepad1.right_stick_x) + gamepad1.left_stick_x) * turnMultiplier);
                config.backLeft.setPower((-gamepad1.right_stick_y - (gamepad1.right_stick_x) + gamepad1.left_stick_x) * turnMultiplier);
                config.frontRight.setPower((-gamepad1.right_stick_y - (gamepad1.right_stick_x) - gamepad1.left_stick_x) * turnMultiplier);
                config.backRight.setPower((-gamepad1.right_stick_y + (gamepad1.right_stick_x) - gamepad1.left_stick_x) * turnMultiplier);
            }

            // Show the elapsed game time and wheel power.
            // telemetry.addData("game1 right stick y", gamepad1.right_stick_y);
            // telemetry.addData("game1 right stick x", gamepad1.right_stick_x);
            // telemetry.addData("game1 left trigger", gamepad1.left_trigger);
            // telemetry.addData("game1 right trigger", gamepad1.right_trigger);
            // telemetry.addData("Status", "Run Time" + runtime);
            telemetry.update();
        }
    }
}