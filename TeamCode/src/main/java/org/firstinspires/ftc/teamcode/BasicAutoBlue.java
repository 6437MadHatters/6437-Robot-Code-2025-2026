package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.BotState.botState;
import org.firstinspires.ftc.teamcode.RRProgs.MecanumDrive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @Disabled // comment this line to show this program on drivers hub
@Autonomous(name = "Basic Auto BLUE", group = "autonomous")
public class BasicAutoBlue extends LinearOpMode {

    // class variables. especially config. And RR.
    Configuration config;
    MecanumDrive drive; //object for roadrunner manipulation
    Methods method;
    BotState state;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // POSITIONS
        // this is where the bot is starting, with the x, y, and rotation (heading (in radians)).
        Pose2d startPose = new Pose2d(50, 52, toRadians(-45));
        Vector2d shootPattern = new Vector2d(36, 36);
        Pose2d shootPatternPose = new Pose2d(shootPattern, toRadians(45));
        Vector2d startIntakeOne = new Vector2d(20,28);
        Pose2d startIntakeOnePose = new Pose2d(startIntakeOne, toRadians(-90));
        Vector2d endIntakeOne = new Vector2d(4, 52);
        Pose2d endIntakeOnePose = new Pose2d(endIntakeOne, toRadians(-90));
        Vector2d startIntakeTwo = new Vector2d(-4, 28);
        Pose2d startIntakeTwoPose = new Pose2d(startIntakeTwo, toRadians(-90));
        Vector2d endIntakeTwo = new Vector2d(-28, 52);
        Pose2d endIntakeTwoPose = new Pose2d(endIntakeOne, toRadians(-90));

        // initialize and assign variables to the other classes with the proper arguments. !!!!! IMPORTANT (must be between positions and actions)
        config = new Configuration(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose); // initialize RR drive with start position
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // TRAJECTORY ACTIONS !!!
        TrajectoryActionBuilder toShootOne = drive.actionBuilder(startPose)
                .strafeToSplineHeading(shootPattern, toRadians(45)); // move from start to shoot position
        TrajectoryActionBuilder toIntakeOne = drive.actionBuilder(shootPatternPose)
                .strafeToSplineHeading(startIntakeOne, toRadians(-90)) // move to prepare for intake
                .strafeToConstantHeading(endIntakeOne, new TranslationalVelConstraint(15)); // intake!
        TrajectoryActionBuilder toShootTwo = drive.actionBuilder(endIntakeOnePose)
                .splineTo(shootPattern, toRadians(45)); // go to shoot again
        TrajectoryActionBuilder toIntakeTwo = drive.actionBuilder(shootPatternPose)
                .strafeToSplineHeading(startIntakeTwo, toRadians(-90)) // move to prepare for intake again
                .strafeToConstantHeading(endIntakeTwo, new TranslationalVelConstraint(15)); // intake again!
        TrajectoryActionBuilder toShootThree = drive.actionBuilder(endIntakeTwoPose)
                .splineTo(shootPattern, toRadians(45)); // go to shoot again again!

        // INIT POSITION
        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false, 0, 45);

        // SCAN APRIL TAG
        // april tag pipeline. you can change these in the limelight manager.
        config.limelight.pipelineSwitch(0); // 0 for blue, 1 for red

        config.limelight.start();
        // Get results from the Limelight
        while (opModeInInit()) {
            telemetry.addLine("Init done. Finding April Tag.");
            LLResult result = config.limelight.getLatestResult();
            int lastTagID = 0;
            if (result != null && result.isValid()){
                lastTagID = result.getFiducialResults().get(0).getFiducialId(); // THIS IS THE TARGET ID!!!!!!!!!!!!!
                telemetry.addData("Detected Tag ID", lastTagID);
            } else {
                telemetry.addData("Limelight", "No targets seen");
            }
            method.updatePattern(lastTagID);
            telemetry.update();
        }

        // run until the auto ends or time runs out (or driver presses STOP)
        boolean running = true;
        waitForStart();
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
                                    state.setBotAction(botState.shoot, false, false, false, false, false),
                                    toShootOne.build(),
                                    state.setBotAction(botState.shoot, true, false,false, false, false), // shoot three times
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.intake, false, false, false, false, false),
                                    toIntakeOne.build(),
                                    // state.setBotAction(botState.intake, false, true, false, false, false), // flick
                                    new ParallelAction(
                                        state.setBotAction(botState.intake, false, false, true, false, false), // scoop
                                        toShootTwo.build()),
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.idle, false, false, false, false, false),
                                    toIntakeTwo.build()
                            ),
                            state.updateStateAction()
                    )
            );
            running = false;
        }
    }
}