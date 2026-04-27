package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RRPoses {

    // this class is just where we define field positions for our autos bot is starting, with the x, y, and rotation (heading (in radians)).
    Methods.alliance currentAlliance; // define the current alliance - this changes when we run an auto
    boolean isThisABackAuto;

    // method to keep RRPoses class currentAlliance variable the same as the one in methods and the runner
    public RRPoses(Telemetry telemetry, Methods.alliance alliance, boolean isThisABackAuto) {
        this.currentAlliance = alliance;
        this.isThisABackAuto = isThisABackAuto;
        telemetry.addData("Alliance", alliance + "Back Auto: " + isThisABackAuto);
    }

    // RED VS BLUE - invert x coordinate and heading for red
    // vector transform
    private Vector2d v(double x, double y) {
        if (currentAlliance == Methods.alliance.red) {
            return new Vector2d(x, -y);
        } else {
            return new Vector2d(x, y);
        }
    }

    // heading transform (orientation - what direction is the robot FACING) RETURNS A DEGREE VALUE, NOT RADIANS YET!!!!!
    public int h(int deg) {
        if (currentAlliance == Methods.alliance.red) {
            return -deg;
        } else {
            return deg;
        }
    }

    // tangent transform (direction - what direction is the robot MOVING)
    public double t(int deg) {
        if (currentAlliance == Methods.alliance.red) {
            return Math.toRadians(-deg);
        } else {
            return Math.toRadians(deg);
        }
    }

    // don't transform pose, every pose is made up of a heading. This method just makes them easier to create.
    public Pose2d p(Vector2d v, int h) {
        return new Pose2d(v, Math.toRadians(h));
    }

    // all of these are functions so that they're only built when they're called. This ensures they're transformed for the right alliance.

    // FRONT AUTOS !!!
    // start
    public Vector2d startFront() {
        return v(62, 38);
    }
    public int startFrontHeading() {
        return h(-90);
    }
    public double startFrontTan() {
        return t(-135); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d startFrontPose() {
        return p(startFront(), startFrontHeading());
    }

    // shoot artifacts
    public Vector2d shootFront() {
        return v(32, 32);
    }
    public int shootFrontHeading() {
        return h(47);
    }
    public double shootFrontTanIn() {
        return t(0); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public double shootFrontTanOut() {
        return t(-165); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d shootFrontPose() {
        return p(shootFront(), shootFrontHeading());
    }
    public Pose2d shootFrontParkPose() {
        return p(endMoveFront(), h(65));
    }

    // start of intake 1
    public Vector2d startIntakeOne() {
        return v(18, 32);
    }
    public int startIntakeOneHeading() {
        return h(-90);
    }
    public double startIntakeOneTan() {
        return t(-140); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d startIntakeOnePose() {
        return p(startIntakeOne(), startIntakeOneHeading());
    }

    // end of intake 1
    public Vector2d endIntakeOne() {
        return v(4, 48);
    }
    public Vector2d endIntakeOneEmpty() {
        return v(1, 55);
    }
    public int endIntakeOneHeading() {
        return h(-90);
    }
    public double endIntakeOneTan() {
        return t(-30); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeOnePose() {
        return p(endIntakeOne(), endIntakeOneHeading());
    }
    public Pose2d endIntakeOneEmptyPose() {
        return p(endIntakeOneEmpty(), endIntakeOneHeading());
    }

    // start of intake 2
    public Vector2d startIntakeTwo() {
        return v(-7, 32);
    }
    public int startIntakeTwoHeading() {
        return h(-90);
        //gay people are named bubbles
    }
    public double startIntakeTwoTan() {
        return t(-140);
    }
    public Pose2d startIntakeTwoPose() {
        return p(startIntakeTwo(), startIntakeTwoHeading());
    }

    // end of intake 2
    public Vector2d endIntakeTwo() {
        return v(-21, 48);
    }
    public int endIntakeTwoHeading() {
        return h(-90);
    }
    public double endIntakeTwoTan() {
        return t(-30); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeTwoPose() {
        return p(endIntakeTwo(), endIntakeTwoHeading());
    }

    // start of intake 3
    public Vector2d startIntakeThree() {
        return v(-33, 32);
    }
    public int startIntakeThreeHeading() {
        return h(-90);
    }
    public double startIntakeThreeTan() {
        return t(-140);
    }
    public Pose2d startIntakeThreePose() {
        return p(startIntakeThree(), startIntakeThreeHeading());
    }

    // end of intake 3
    public Vector2d endIntakeThree() {
        return v(-47, 48);
    }
    public int endIntakeThreeHeading() {
        return h(-90);
    }
    public double endIntakeThreeTan() {
        return t(-30); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeThreePose() {
        return p(endIntakeThree(), endIntakeThreeHeading());
    }

    // empty???
    public Vector2d empty() {
        return v(2, 55);
    }
    public int emptyHeading() {
        return h(0);
    }
    public double emptyTan() {
        return t(45); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d emptyPose() {
        return p(empty(), emptyHeading());
    }

    // end
    public Vector2d endMoveFront() {
        return v(43, 23);
    }
    public int endMoveFrontHeading() {
        return h(45);
    }
    public double endMoveFrontTan() {
        return t(-45); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endMoveFrontPose() {
        return p(endMoveFront(), endMoveFrontHeading());
    }

    // BACK AUTOS !!!
    // start
    public Vector2d startBack() {
        return v(-62, 16);
    }
    public int startBackHeading() {
        return h(0);
    }
    public double startBackTan() {
        return t(90); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d startBackPose() {
        return p(startBack(), startBackHeading());
    }

    // shoot one artifact
    public Vector2d shootBack() {
        return v(-58, 15);
    }
    public int shootBackHeading() {
        return h(21);
    }
    public double shootBackTanIn() {
        return t(-90); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public double shootBackTanOut() {
        return t(60); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d shootBackPose() {
        return p(shootBack(), shootBackHeading());
    }

    // start of row intake
    public Vector2d startIntakeRow() {
        return v(-30, 29);
    }
    public int startIntakeRowHeading() {
        return h(-90);
    }
    public double startIntakeRowTan() {
        return t(-50);
    }
    public Pose2d startIntakeRowPose() {
        return p(startIntakeRow(), startIntakeRowHeading());
    }

    // end of row intake
    public Vector2d endIntakeRow() {
        return v(-45, 52);
    }
    public int endIntakeRowHeading() {
        return h(-90);
    }
    public double endIntakeRowTan() {
        return t(-120); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeRowPose() {
        return p(endIntakeRow(), endIntakeRowHeading());
    }

    // end move
    public Vector2d endBack() {
        return v(-58, 32);
    }
    public int endBackHeading() {
        return h(-90);
    }
    public double endBackTan() {
        return t(90); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endBackPose() {
        return p(endIntakeRow(), endIntakeRowHeading());
    }
}
