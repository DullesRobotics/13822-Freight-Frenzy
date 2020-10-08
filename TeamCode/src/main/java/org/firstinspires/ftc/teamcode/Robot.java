package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareHandlers.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareHandlers.Controller;
import org.firstinspires.ftc.teamcode.HardwareHandlers.Servo;
import org.firstinspires.ftc.teamcode.HardwareHandlers.TouchSensor;
import org.firstinspires.ftc.teamcode.HardwareHandlers.motor.Motor;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Hello, code begins here :D
 */
public class Robot {

    private volatile LinearOpMode op;
    private volatile Controller controller1, controller2;
    private volatile ArrayList<Motor> motors = new ArrayList<>();
    private volatile ArrayList<Servo> servos = new ArrayList<>();
    private volatile ArrayList<TouchSensor> touchSensors = new ArrayList<>();
    private volatile ArrayList<ColorSensor> colorSensors = new ArrayList<>();

    public Robot(LinearOpMode op){
        this.op = op;
        this.controller1 = new Controller(op.gamepad1);
        this.controller2 = new Controller(op.gamepad2);
    }

    public void addMotors(Motor... motors){
        this.motors.addAll(Arrays.asList(motors));
    }

    public void addServos(Servo... servos){
        this.servos.addAll(Arrays.asList(servos));
    }

    public void addTouchSensors(TouchSensor... touchSensors){ this.touchSensors.addAll(Arrays.asList(touchSensors)); }

    public void addColorSensors(ColorSensor... colorSensors){ this.colorSensors.addAll(Arrays.asList(colorSensors)); }



}
