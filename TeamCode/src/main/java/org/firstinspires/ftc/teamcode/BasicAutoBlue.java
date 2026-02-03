package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.BotState.botState;
import org.firstinspires.ftc.teamcode.RRProgs.MecanumDrive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

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
        // this is where the bot is starting, with the x, y, and rotation (heading (in radians(3189 put tape on the field labeling this))).
        Pose2d startPose = new Pose2d(50, 52, toRadians(-40));
        Vector2d shootThree = new Vector2d(20, 20);
        Pose2d shootThreePose = new Pose2d(shootThree, toRadians(45));
        Vector2d startIntakeOne = new Vector2d(12,12);
        Pose2d startIntakeOnePose = new Pose2d(startIntakeOne, toRadians(-90));
        Vector2d endIntakeOne = new Vector2d(12, 46);
        Pose2d endIntakeOnePose = new Pose2d(endIntakeOne, toRadians(-90));

        // initialize and assign variables to the other classes with the proper arguments. !!!!! IMPORTANT (must be between positions and actions)
        config = new Configuration(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose); // initialize RR drive with start position
        method = new Methods(config);
        state = new BotState(config, method, telemetry);

        // TRAJECTORY ACTIONS !!!
        TrajectoryActionBuilder outToShoot = drive.actionBuilder(startPose)
                .strafeToLinearHeading(shootThree, toRadians(45)); // move from start to shoot position
        TrajectoryActionBuilder toIntake = drive.actionBuilder(shootThreePose)
                .strafeToLinearHeading(startIntakeOne, toRadians(-90)) // move to prepare for intake
                .strafeToConstantHeading(endIntakeOne); // intake!

        // INIT POSITION
        state.setBot(botState.idle);
        state.updateBotState(false, false, false, false);

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

            // follow trajectory
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    state.setBotAction(botState.slowSpeed, false, false, false, false, false),
                                    outToShoot.build(),
                                    state.setBotAction(botState.slowSpeed, true, false,false, false, false), // shoot three times
                                    state.setBotAction(botState.slowSpeed, true, false, false, false, false),
                                    state.setBotAction(botState.slowSpeed, true, false, false, false, false),
                                    state.setBotAction(botState.intake, false, false, false, false, false),
                                    toIntake.build(),
                                    state.setBotAction(botState.idle, false, false, false, false, false)
                            ),
                            state.updateStateAction()
                    )
            );
            running = false;
        }
    }
}