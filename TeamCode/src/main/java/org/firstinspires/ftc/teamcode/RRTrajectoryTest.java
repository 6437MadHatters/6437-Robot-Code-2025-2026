package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.BotState.botState;
import org.firstinspires.ftc.teamcode.RRProgs.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled // comment this line to show this program on drivers hub
@Autonomous(name = "RR Trajectory Test", group = "autonomous")
public class RRTrajectoryTest extends LinearOpMode {

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
        // this is where the bot is starting, with the x, y, and rotation (heading (in radians(3189 put tape on the field labeling this))).
        Pose2d startPose = new Pose2d(12, 12, toRadians(0));
        Vector2d corner1 = new Vector2d(12, 12);
        Pose2d corner1Pose = new Pose2d(corner1, toRadians(0));
        Vector2d corner2 = new Vector2d(60, 12);
        Pose2d corner2Pose = new Pose2d(corner2, toRadians(90));
        Vector2d corner3 = new Vector2d(60,60);
        Pose2d corner3Pose = new Pose2d(corner3, toRadians(180));
        Vector2d corner4 = new Vector2d(12,60);
        Pose2d corner4Pose = new Pose2d(corner4, toRadians(270));

        // initialize and assign variables to the other classes with the proper arguments. !!!!! IMPORTANT
        config = new Configuration(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose); // initialize RR drive with start position
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // TRAJECTORY ACTIONS !!!
        TrajectoryActionBuilder strafeAround = drive.actionBuilder(startPose)
                .strafeToLinearHeading(corner2, toRadians(90))
                .strafeToLinearHeading(corner3, toRadians(180))
                .strafeToLinearHeading(corner4, toRadians(270))
                .strafeToLinearHeading(corner1, toRadians(0));

        TrajectoryActionBuilder strafeAroundSideOne = drive.actionBuilder(corner1Pose)
                .strafeToLinearHeading(corner2, toRadians(90));
        TrajectoryActionBuilder strafeAroundSideTwo = drive.actionBuilder(corner2Pose)
                .strafeToLinearHeading(corner3, toRadians(180));
        TrajectoryActionBuilder strafeAroundSideThree = drive.actionBuilder(corner3Pose)
                .strafeToLinearHeading(corner4, toRadians(270));
        TrajectoryActionBuilder strafeAroundSideFour = drive.actionBuilder(corner4Pose)
                .strafeToLinearHeading(corner1, toRadians(0));

        TrajectoryActionBuilder strafeAroundLineOne = drive.actionBuilder(corner1Pose)
                .strafeToLinearHeading(corner2, toRadians(180));
        TrajectoryActionBuilder strafeAroundLineTwo = drive.actionBuilder(corner2Pose)
                .strafeToLinearHeading(corner1, toRadians(0));

        TrajectoryActionBuilder turnTest = drive.actionBuilder(startPose)
                .turn(toRadians(180));

        // INIT POSITION
        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false, 0, 45);

        // run until the auto ends or time runs out (or driver presses STOP)
        boolean running = true;
        waitForStart();
        while (opModeIsActive() && running) {

            // follow trajectory
                Actions.runBlocking(
                        new ParallelAction(
                            new SequentialAction(
                                    state.setBotAction(botState.shoot, false, false, false, false, false),
                                    strafeAroundSideOne.build(),
                                    state.setBotAction(botState.shoot, false, true, false, false, false),
                                    //state.setBotAction(botState.idle, false, false, false, false),
                                    strafeAroundSideTwo.build(),
                                    state.setBotAction(botState.intake, false, false, false, false, false),
                                    strafeAroundSideThree.build(),
                                    state.setBotAction(botState.idle, false, false, false, false, false),
                                    strafeAroundSideFour.build()
                            ),
                        state.updateStateAction()
                        )
                );
                running = false;
        }
    }
}