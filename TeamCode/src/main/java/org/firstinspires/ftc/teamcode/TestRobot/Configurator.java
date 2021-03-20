package org.firstinspires.ftc.teamcode.TestRobot;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

public class Configurator {

    /**
     * It's HIGHLY recommended every motor has the same motor configuration for autonomous driving
     * If not, individual motors might move incorrectly. If you have a mix-match of motors, don't use road runner.
     * @return The main hardware list for the robot
     */
    public static HardwareComponent[] getHardware(Robot robot){

        MotorConfiguration MC = new MotorConfiguration(
                MotorType.NEVEREST_ORBITAL,
                true,
                2.9528,
                1);

        DrivetrainMotor motorFrontLeft = new DrivetrainMotor(robot, "FLM", MC, true, MotorType.DrivetrainPosition.FLM);
        DrivetrainMotor motorFrontRight = new DrivetrainMotor(robot, "FRM", MC, true, MotorType.DrivetrainPosition.FRM);
        DrivetrainMotor motorBackLeft = new DrivetrainMotor(robot, "BLM", MC, true, MotorType.DrivetrainPosition.BLM);
        DrivetrainMotor motorBackRight = new DrivetrainMotor(robot, "BRM", MC, true, MotorType.DrivetrainPosition.BRM);

        return(new HardwareComponent[]{
                motorFrontLeft,
                motorFrontRight,
                motorBackLeft,
                motorBackRight,
//                new Servo(robot, "CL", HardwareComponentArea.CLAW),
//                new Servo(robot, "IN", HardwareComponentArea.INTAKE),
//                new TouchSensor(robot, "TC", HardwareComponentArea.CLAW),
//                new TouchSensor(robot, "TI", HardwareComponentArea.INTAKE),
//                new ColorSensor(robot, "CC", HardwareComponentArea.CLAW),
//                new ColorSensor(robot, "CI", HardwareComponentArea.INTAKE),
                new IMU(robot, "IMU"),
                //new USBWebcam(robot, "Webcam")
        });
    }

}
