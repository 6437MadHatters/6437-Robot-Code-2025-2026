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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @Disabled // comment this line to show this program on drivers hub
@Autonomous(name = "Basic Auto RED", group = "autonomous")
public class BasicAutoRed extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(56, -56, toRadians(35));
        Vector2d shootThree = new Vector2d(26, -26);
        Pose2d shootThreePose = new Pose2d(shootThree, toRadians(-60));
        Vector2d moveOut = new Vector2d(10,-24);
        Pose2d moveOutPose = new Pose2d(moveOut, toRadians(-60));

        // initialize and assign variables to the other classes with the proper arguments. !!!!! IMPORTANT
        config = new Configuration(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose); // initialize RR drive with start position
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // TRAJECTORY ACTIONS !!!
        TrajectoryActionBuilder outToShoot = drive.actionBuilder(startPose)
                .strafeToLinearHeading(shootThree, toRadians(-60)); // move from start to shoot
        TrajectoryActionBuilder offLine = drive.actionBuilder(shootThreePose)
                .strafeToConstantHeading(moveOut); // move off the line

        // INIT POSITION
        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false, 0, 45);

        config.limelight.start();
        // Get results from the Limelight
        while (opModeInInit()) {
            telemetry.addLine("Init done. Finding April Tag.");
            LLResult result = config.limelight.getLatestResult();
            if (result != null && result.isValid()){
                int lastTagID = result.getFiducialResults().get(0).getFiducialId(); // THIS IS THE TARGET ID!!!!!!!!!!!!!
                telemetry.addData("Detected Tag ID", lastTagID);
            } else {
                telemetry.addData("Limelight", "No targets seen");
            }
            telemetry.update();
        }

        // run until the auto ends or time runs out (or driver presses STOP)
        boolean running = true;
        waitForStart();
        while (opModeIsActive() && running) {

            // follow trajectory
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    state.setBotAction(botState.shoot, false, false, false, false, false),
                                    outToShoot.build(),
                                    state.setBotAction(botState.shoot, true, false, false, false, false),
                                    state.setBotAction(botState.idle, false, false, false, false, false),
                                    offLine.build()
                            ),
                            state.updateStateAction()
                    )
            );
            running = false;
        }
    }
}