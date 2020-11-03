package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class Controller {

    private Gamepad g;

    public Controller(Gamepad g){
        this.g = g;
    }

    public boolean buttonA() {
        return g.a;
    }

    public boolean buttonB() {
        return g.b;
    }

    public boolean buttonX() {
        return g.x;
    }

    public boolean buttonY() {
        return g.y;
    }

    public boolean buttonUp() {
        return g.dpad_up;
    }

    public boolean buttonDown() {
        return g.dpad_down;
    }

    public boolean buttonRight() {
        return g.dpad_right;
    }

    public boolean buttonLeft() {
        return g.dpad_left;
    }

    public boolean leftBumper() { return g.left_bumper; }

    public boolean rightBumper() {
        return g.right_bumper;
    }

    public float leftTrigger(){
        float preReading = g.left_trigger;
        float reading = (float)(Math.pow(g.left_trigger,2));
        if(preReading < 0)
            reading*= -1;
        return reading;
    }

    public float rightTrigger(){
        float preReading = g.right_trigger;
        float reading = (float)(Math.pow(g.right_trigger,2));
        if(preReading < 0)
            reading*= -1;
        return reading;
    }

    public float leftX(){ return g.left_stick_x; }

    public float rightX(){
        return g.right_stick_x;
}

    public float leftY(){
        return  g.left_stick_y;
    }

    public float rightY(){
        return g.right_stick_y;
    }

    public boolean leftStickButton(){
        return g.left_stick_button;
    }

    public boolean rightStickButton(){
        return g.right_stick_button;
    }

    public boolean buttonStart() {
        return g.start;
    }

    public boolean buttonBack() {
        return g.back;
    }

    public boolean buttonMode(){
        return g.guide;
    }

//    /** Get a boolean reading */
//    public boolean isPressed(Button b){
//        switch (b) {
//            case A: return buttonA();
//            case B: return buttonB();
//            case X: return buttonX();
//            case Y: return buttonY();
//            case D_UP: return buttonUp();
//            case D_DOWN: return buttonDown();
//            case D_RIGHT: return buttonRight();
//            case D_LEFT: return buttonLeft();
//            case BUMPER_LEFT: return leftBumper();
//            case BUMPER_RIGHT: return rightBumper();
//            case STICK_LEFT_B: return leftStickButton();
//            case STICK_RIGHT_B: return rightStickButton();
//            case START: return buttonStart();
//            case BACK: return buttonBack();
//            case MODE: return buttonMode();
//            default: return false;
//        }
//    }
//
//    /** Get a float reading */
//    public float getReading(Reading r){
//        switch(r){
//            case STICK_LEFT_X: return leftX();
//            case STICK_LEFT_Y: return leftY();
//            case STICK_RIGHT_X: return rightX();
//            case STICK_RIGHT_Y: return rightY();
//            case TRIGGER_LEFT: return leftTrigger();
//            case TRIGGER_RIGHT: return rightTrigger();
//            default: return 0;
//        }
//    }
//
//    /** Controller inputs that are pressed or not pressed */
//    public enum Button {
//        A,
//        B,
//        X,
//        Y,
//        D_UP,
//        D_DOWN,
//        D_RIGHT,
//        D_LEFT,
//        BUMPER_LEFT,
//        BUMPER_RIGHT,
//        STICK_LEFT_B,
//        STICK_RIGHT_B,
//        START,
//        BACK,
//        MODE
//    }
//
//    /** Controller inputs that return float readings */
//    public enum Reading {
//        STICK_LEFT_X,
//        STICK_LEFT_Y,
//        STICK_RIGHT_X,
//        STICK_RIGHT_Y,
//        TRIGGER_LEFT,
//        TRIGGER_RIGHT
//    }
//
//    public HashMap<String, String> getState(){
//
//    }

}
