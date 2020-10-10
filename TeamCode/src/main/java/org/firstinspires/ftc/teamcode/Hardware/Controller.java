package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

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

    public double leftTrigger(){
        double preReading = g.left_trigger;
        float reading = (float)(Math.pow(g.left_trigger,2));
        if(preReading < 0)
            reading*= -1;
        return reading;
    }

    public double rightTrigger(){
        double preReading = g.right_trigger;
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

    @Deprecated
    public boolean buttonBack() {
        return g.back;
    }

    @Deprecated
    public boolean guide(){
        return g.guide;
    }

}
