package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.HardwareHandlers.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareHandlers.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.HardwareHandlers.Servo;
import org.firstinspires.ftc.teamcode.HardwareHandlers.TouchSensor;
import org.firstinspires.ftc.teamcode.HardwareHandlers.motor.Motor;

public class OpModeHandler {

    public static void addHardware(Robot robot, LinearOpMode op){
        robot.addMotors(
                new Motor(op, "LF", HardwareComponentArea.DRIVE),
                new Motor(op, "RF", HardwareComponentArea.DRIVE),
                new Motor(op, "LB", HardwareComponentArea.DRIVE),
                new Motor(op, "RB", HardwareComponentArea.DRIVE));

        robot.addServos(
                new Servo(op, "CL", HardwareComponentArea.CLAW),
                new Servo(op, "IN", HardwareComponentArea.INTAKE));

        robot.addTouchSensors(
                new TouchSensor(op, "TC", HardwareComponentArea.CLAW),
                new TouchSensor(op, "TI", HardwareComponentArea.INTAKE));

        robot.addColorSensors(
                new ColorSensor(op, "CC", HardwareComponentArea.CLAW),
                new ColorSensor(op, "CI", HardwareComponentArea.INTAKE));
    }

}
