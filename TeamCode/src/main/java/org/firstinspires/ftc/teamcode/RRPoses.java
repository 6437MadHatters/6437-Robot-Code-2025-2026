package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RRPoses {

    // this class is just where we define field positions for our autos bot is starting, with the x, y, and rotation (heading (in radians)).
    Methods.alliance currentAlliance; // define the current alliance - this changes when we run an auto

    // method to keep RRPoses class currentAlliance variable the same as the one in methods and the runner
    public RRPoses (Telemetry telemetry, Methods.alliance alliance) {
        this.currentAlliance = alliance;
        telemetry.addData("Poses built with", alliance);
    }

    // RED VS BLUE - invert x coordinate and heading for red
    // vector transform
    private Vector2d v (double x, double y){
        if (currentAlliance == Methods.alliance.red) {
            return new Vector2d(x, -y);
        } else {
            return new Vector2d(x, y);
        }
    }
    // heading transform (orientation - what direction is the robot FACING)
    private int h (int deg) {
        if (currentAlliance == Methods.alliance.red) {
            return -deg;
        } else {
            return deg;
        }
    }
    // tangent transform (direction - what direction is the robot MOVING)
    private double t (int deg) {
        if (currentAlliance == Methods.alliance.red) {
            return Math.toRadians(-deg);
        } else {
            return Math.toRadians(deg);
        }
    }
    // don't transform pose, every pose is made up of a heading. This method just makes them easier to create.
    private Pose2d p (Vector2d v, int h) {
        return new Pose2d(v, Math.toRadians(h));
    }

    // all of these are functions so that they're only built when they're called. This ensures they're transformed for the right alliance.

    // FRONT AUTOS !!!
    // start
    public Vector2d startFront(){
        return v(62,38);
    }
    public int startFrontHeading() {
        return h(-90);
    }
    public double startFrontTan(){
        return t(-135); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d startFrontPose(){
        return p (startFront(), startFrontHeading());
    }

    // shoot one artifact
    public Vector2d shoot(){
        return v(32, 32);
    }
    public int shootHeading(){
        return h(45);
    } // we did the math
    public double shootTanIn(){
        return t(0); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public double shootTanOut(){
        return t(-165); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d shootPose(){
        return p(shoot(), shootHeading());
    }

    // start of intake 1
    public Vector2d startIntakeOne(){
        return v(18, 31);
    }
    public int startIntakeOneHeading(){
        return h(-90);
    }
    public double startIntakeOneTan(){
        return t(-135); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d startIntakeOnePose(){
        return p(startIntakeOne(), startIntakeOneHeading());
    }

    // end of intake 1
    public Vector2d endIntakeOne(){
        return v(4, 48);
    }
    public int endIntakeOneHeading() {
        return h(-90);
    }
    public double endIntakeOneTan(){
        return t(-30); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeOnePose() {
        return p(endIntakeOne(), endIntakeOneHeading());
    }

    // start of intake 2
    public Vector2d startIntakeTwo(){
        return v(-8, 31);
    }
    public int startIntakeTwoHeading(){
        return h(-90);
    }
    public double startIntakeTwoTan(){
        return t(-135);
    }
    public Pose2d startIntakeTwoPose(){
        return p(startIntakeTwo(), startIntakeTwoHeading());
    }

    // end of intake 2
    public Vector2d endIntakeTwo(){
        return v(-22, 48);
    }
    public int endIntakeTwoHeading(){
        return h(-90);
    }
    public double endIntakeTwoTan(){
        return t(-30); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endIntakeTwoPose(){
        return p(endIntakeTwo(), endIntakeTwoHeading());
    }

    // end
    public Vector2d endMoveFront(){
        return v(38, 26);
    }
    public int endMoveFrontHeading(){
        return h(45);
    }
    public double endMoveFrontTan(){
        return t(-45); // SUBJECT TO CHANGE BASED ON MOVEMENT
    }
    public Pose2d endMoveFrontPose(){
        return p(endMoveFront(), endMoveFrontHeading());
    }

    // BACK AUTOS !!!


    // quick check to see if y switched to negative for red
    public void check(Telemetry telemetry) {
        telemetry.addData("start Vector2d", startFront());
    }

}
