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
@Autonomous(name = "Front Pattern", group = "autonomous")
public class frontPattern extends LinearOpMode {

    // class variables. especially config. And RR.
    Configuration config;
    MecanumDrive drive; //object for roadrunner manipulation
    Methods method;
    BotState state;
    RRPoses poses;

    int shootRPM = 2100;
    int shootAngle = 73;

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
            method.readTagIDs(telemetry, false); // THIS IS NOT A BACK AUTO -> THIS MUST BE FALSE
            telemetry.addData("Colors (left, mid, right)", method.getBallColor(config.colorLeft, config.rangeLeft) + ", " + method.getBallColor(config.colorMid, config.rangeMid) + ", " + method.getBallColor(config.colorRight, config.rangeRight));
            //telemetry.addData("Ranges (left, mid, right)", method.getRange(config.rangeLeft) + ", " + method.getRange(config.rangeMid) + ", " + method.getRange(config.rangeRight));
            //telemetry.addData("Hues (left, mid, right)", method.getHue(config.colorLeft) + ", " + method.getHue(config.colorMid) + ", " + method.getHue(config.colorRight));
            telemetry.update();
        }

        // initialize and assign more variables to the other classes with the proper arguments. these need to know what the alliance tag is.
        poses = new RRPoses(telemetry, method.getCurrentAlliance(), false); // build with correct alliance IS THIS A BACK AUTO? NO! FALSE!
        drive = new MecanumDrive(hardwareMap, poses.startFrontPose()); // initialize RR drive with start position (it changes after we scan)

        // TRAJECTORY ACTIONS !!! these are the paths the robot actually follows***
        TrajectoryActionBuilder toShootOne = drive.actionBuilder(poses.startFrontPose())
                .setTangent(poses.startFrontTan())
                .splineToSplineHeading(poses.shootFrontPose(), poses.shootFrontTanOut(), new AngularVelConstraint(Math.PI/2)); // move from start to shoot position (tangent is weird)

        TrajectoryActionBuilder toIntakeOne = drive.actionBuilder(poses.shootFrontPose())
                .setTangent(poses.shootFrontTanOut())
                .splineToSplineHeading(poses.startIntakeOnePose(), -poses.startIntakeOneTan()) // move to prepare for intake
                .strafeToConstantHeading(poses.endIntakeOne(), new TranslationalVelConstraint(17)); // intake!

        TrajectoryActionBuilder toShootTwo = drive.actionBuilder(poses.endIntakeOnePose())
                .setTangent(poses.endIntakeOneTan())
                .splineToSplineHeading(poses.shootFrontPose(), poses.shootFrontTanIn()); // go to shoot again

        TrajectoryActionBuilder toIntakeTwo = drive.actionBuilder(poses.shootFrontPose())
                .setTangent(poses.shootFrontTanOut())
                .splineToSplineHeading(poses.startIntakeTwoPose(), -poses.startIntakeTwoTan()) // move to prepare for intake again
                .strafeToConstantHeading(poses.endIntakeTwo(), new TranslationalVelConstraint(17)); // intake again!

        TrajectoryActionBuilder toShootThree = drive.actionBuilder(poses.endIntakeTwoPose())
                .setTangent(poses.endIntakeTwoTan())
                .splineToSplineHeading(poses.shootFrontPose(), poses.shootFrontTanIn()); // go to shoot again again!

        TrajectoryActionBuilder toEndMove = drive.actionBuilder(poses.shootFrontPose())
                .strafeToConstantHeading(poses.endMoveFront()); // get off line to get move points

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
                                    state.setBotAction(botState.shoot, false, false, false, shootRPM, shootAngle), // spin up when bot starts
                                    toShootOne.build(),
                                    state.setBotAction(botState.shoot, true, false,false, shootRPM, shootAngle), // shoot three times
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.intake, false, false, false, 0, 0),
                                    toIntakeOne.build(),
                                    new ParallelAction( // scoop once and start spinning up on the way to shoot
                                            new SequentialAction(
                                                    state.setBotAction(botState.intake, false, false, true, 0, 0),
                                                    new SleepAction(.25), // wait for a stuck ball to come up
                                                    state.setBotAction(botState.shoot, false, false, false, shootRPM, shootAngle)),
                                            toShootTwo.build()),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.intake, false, false, false, 0, 0),
                                    toIntakeTwo.build(),
                                    new ParallelAction( // scoop once and start spinning up on the way to shoot
                                            new SequentialAction(
                                                    state.setBotAction(botState.intake, false, false, true, 0, 0),
                                                    new SleepAction(.25), // wait for a stuck ball to come up
                                                    state.setBotAction(botState.shoot, false, false, false, 2100, shootAngle)),
                                            toShootThree.build()),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),
                                    state.setBotAction(botState.shoot, true, false, false, shootRPM, shootAngle),

                                    state.setBotAction(botState.idle, false, false, false, 0, 0),
                                    toEndMove.build()
                            ),
                            state.updateStateAction()
                    )
            );
            running = false;
        }
    }
}