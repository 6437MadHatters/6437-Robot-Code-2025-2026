package org.firstinspires.ftc.teamcode;

//This class is designed to make the buttons not stupid. This is meant mostly so you can check if the button press is new (just pressed) or has been held. This is super useful for a toggle that is meant to only trigger on the fresh press of the button

public class Button {
    //represents the current state of the button
    private boolean button;
    //this checks if the button is pressed currently
    private boolean stillPressed = false;
    //if the button was pressed in the last loop
    private boolean pressedLastLoop = false;
    //basic constructor, just pass it the button that the object will represent.
    public Button(boolean button) {
        this.button = button;
    }
    //just getter methods
    public boolean pressedLastLoop() {
        return pressedLastLoop;
    }

    public boolean isPressed() {
        return stillPressed;
    }
    //just does some stuff to make sure update all of the boolean variables
    public void update(boolean button) {
        this.button = button;
        if (this.button && !stillPressed) {
            stillPressed = true;
        } else if (this.button && !pressedLastLoop) {
            pressedLastLoop = true;
        } else if (!this.button && stillPressed) {
            stillPressed = false;
        } else if (pressedLastLoop && !this.button) {
            pressedLastLoop = false;
        }
    }
    //this method only returns true if it is a new button press, and not a button press that has been held.
    //this is the method that you should call if you want to see if a button has just been pressed
    public boolean buttonPress() {
        return isPressed() && !pressedLastLoop();
    }
}
