package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.RRProgs.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled // comment this line to show this program on drivers hub
@Autonomous(name = "RR Trajectory Test", group = "autonomous")
public class RRTrajectoryTest extends LinearOpMode {

    // class variables. especially config. And RR.
    Configuration config;
    MecanumDrive drive; //object for roadrunner manipulation

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // POSITIONS
        // this is where the bot is starting, with the x, y, and rotation (heading (in radians(3189 put tape on the field labeling this))).
        Pose2d startPose = new Pose2d(12, 12, toRadians(0));
        Vector2d corner1 = new Vector2d(12, 12);
        Vector2d corner2 = new Vector2d(60, 12);
        Vector2d corner3 = new Vector2d(60,60);
        Vector2d corner4 = new Vector2d(12,60);

        // initialize and assign variables to the other classes with the proper arguments. !!!!! IMPORTANT
        config = new Configuration(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose); // initialize RR drive with start position

        // TRAJECTORY ACTIONS !!!
        TrajectoryActionBuilder strafeAroundConstantHeading = drive.actionBuilder(startPose)
                .strafeToLinearHeading(corner2, toRadians(90))
                .strafeToLinearHeading(corner3, toRadians(180))
                .strafeToLinearHeading(corner4, toRadians(270))
                .strafeToLinearHeading(corner1, toRadians(0));

        // CUSTOM ACTIONS (presets) !!!
        // nothing...

        // run until the auto ends or time runs out (or driver presses STOP)
        waitForStart();
        while (opModeIsActive()) {

            // follow trajectory
            Actions.runBlocking(
                    strafeAroundConstantHeading.build());
        }
    }
}