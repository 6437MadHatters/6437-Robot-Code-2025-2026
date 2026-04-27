package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.BotState.botState;
import org.firstinspires.ftc.teamcode.RRProgs.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @Disabled // comment this line to show this program on drivers hub
@Autonomous(name = "Back 1 Cycle", group = "autonomous")
public class back1Cycle extends LinearOpMode {

    // class variables. especially config. And RR.
    Configuration config;
    MecanumDrive drive; // object for roadrunner manipulation
    Methods method;
    BotState state;
    RRPoses poses;

    int shootRPM = 3100;
    int shootAngle = 45;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize and assign variables to the other classes with the proper arguments.
        config = new Configuration(hardwareMap);
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // initialize limelight
        config.limelight.pipelineSwitch(0); // pipeline 0 for autos
        config.limelight.start();

        // INIT POSITION
        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false, false,0, 45);

        // Get results from the Limelight - short init loop
        while (opModeInInit()) {
            telemetry.addLine("Init done. Finding April Tag.");
            method.readTagIDs(telemetry, true); // THIS IS A BACK AUTO -> THIS MUST BE TRUE
            telemetry.addData("Colors (left, mid, right)", method.getBallColor(config.colorLeft, config.rangeLeft) + ", " + method.getBallColor(config.colorMid, config.rangeMid) + ", " + method.getBallColor(config.colorRight, config.rangeRight));
            //telemetry.addData("Ranges (left, mid, right)", method.getRange(config.rangeLeft) + ", " + method.getRange(config.rangeMid) + ", " + method.getRange(config.rangeRight));
            //telemetry.addData("Hues (left, mid, right)", method.getHue(config.colorLeft) + ", " + method.getHue(config.colorMid) + ", " + method.getHue(config.colorRight));
            telemetry.update();
        }

        // initialize and assign more variables to the other classes with the proper arguments. these need to know what the alliance tag is.
        poses = new RRPoses(telemetry, method.getCurrentAlliance(), true); // build with correct alliance IS THIS A BACK AUTO? YES! TRUE!
        drive = new MecanumDrive(hardwareMap, poses.startBackPose()); // initialize RR drive with start position (it changes after we scan)

        // TRAJECTORY ACTIONS !!! these are the paths the robot actually follows***
        TrajectoryActionBuilder toShootOne = drive.actionBuilder(poses.startBackPose())
                //.setTangent(poses.startBackTan())
                .strafeToSplineHeading(poses.shootBack(), Math.toRadians(poses.shootBackHeading()), new AngularVelConstraint(Math.PI/2)); // strafe: don't forget to convert heading to radians

        TrajectoryActionBuilder toEndMove = drive.actionBuilder(poses.shootBackPose())
                .splineToSplineHeading(poses.endBackPose(), poses.endBackHeading()); // get off line to get move points

        // run until the auto ends or time runs out (or driver presses STOP)
        waitForStart();
        boolean running = true;
        while (opModeIsActive() && running) {

            // KEY FOR BOOLEANS
            // usePattern = only for pattern when shooting
            // allA = shoot all or flick
            // rightB = shoot right or scoop
            // leftX = shoot left
            // middleY = shoot middle

            // follow trajectory
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    state.setBotAction(botState.shoot, false, false, false, shootRPM, shootAngle), // start flywheel while bot is moving
                                    toShootOne.build(),
                                    state.setBotAction(botState.shoot, true, false,false, shootRPM, shootAngle), // shoot a ball
                                    state.setBotAction(botState.shoot, true, false,false, shootRPM, shootAngle), // shoot a ball
                                    state.setBotAction(botState.shoot, true, false,false, shootRPM, shootAngle), // shoot a ball
                                    state.setBotAction(botState.intake, false, false, false, 0, 0), // stop flywheel, start intake

                                    state.setBotAction(botState.idle, false, false, false, 0, 0), // stop flywheel, move out the way
                                    toEndMove.build()
                            ),
                            state.updateStateAction()
                    )
            );
            running = false;
        }
    }
}