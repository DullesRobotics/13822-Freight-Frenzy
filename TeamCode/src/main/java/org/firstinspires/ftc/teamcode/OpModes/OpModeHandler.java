package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor;

public class OpModeHandler {

    /**
     * Add the hardware to the robot class
     * @param robot The robot this is configuring for
     * @param op The current op mode
     */
    public static void addHardware(Robot robot, LinearOpMode op){

        MotorConfiguration motorConfiguration = new MotorConfiguration(MotorConfiguration.MotorType.CORE_HEX_MOTOR, 3);
        Motor parentMotor = new Motor(op, "LF", HardwareComponentArea.DRIVE, motorConfiguration, true, true);

        robot.addHardware(
                parentMotor, //left front motor
                parentMotor.clone("RF"), //right front motor
                parentMotor.clone("LB"), //left back motor
                parentMotor.clone("RB"), //right back motor
                new Servo(op, "CL", HardwareComponentArea.CLAW),
                new Servo(op, "IN", HardwareComponentArea.INTAKE),
                new TouchSensor(op, "TC", HardwareComponentArea.CLAW),
                new TouchSensor(op, "TI", HardwareComponentArea.INTAKE),
                new ColorSensor(op, "CC", HardwareComponentArea.CLAW),
                new ColorSensor(op, "CI", HardwareComponentArea.INTAKE));
    }

}
