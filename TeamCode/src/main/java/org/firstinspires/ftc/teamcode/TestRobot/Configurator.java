package org.firstinspires.ftc.teamcode.TestRobot;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.HashMap;

public class Configurator {

    private final static String ID_frontLeft = "FLM", ID_frontRight = "FRM", ID_backLeft = "BLM", ID_backRight = "BRM";


    /**
     * It's HIGHLY recommended every motor has the same motor configuration for autonomous driving
     * If not, individual motors might move incorrectly. If you have a mix-match of motors, don't use road runner.
     * @return The main hardware list for the robot
     */
    public static HardwareComponent[] getHardware(Robot r){
        MotorConfiguration mC = new MotorConfiguration(
                MotorType.NEVEREST_ORBITAL,
                false,
                true,
                2.9528,
                1);

        Motor motorFrontLeft = new Motor(r, ID_frontLeft, HardwareComponentArea.DRIVE_TRAIN, mC, false),
        motorFrontRight = new Motor(r, ID_frontRight, HardwareComponentArea.DRIVE_TRAIN, mC, true),
        motorBackLeft = motorFrontLeft.clone(ID_backLeft),
        motorBackRight = motorFrontRight.clone(ID_backRight);

        //front left & back right are strafe opposite
        motorBackLeft.setStrafeOpposite(false);
        motorBackRight.setStrafeOpposite(true);

        return(new HardwareComponent[]{
                motorFrontLeft, //left front motor
                motorFrontRight, //right front motor
                motorBackLeft, //left back motor
                motorBackRight, //right back motor
//                new Servo(r, "CL", HardwareComponentArea.CLAW),
//                new Servo(r, "IN", HardwareComponentArea.INTAKE),
//                new TouchSensor(r, "TC", HardwareComponentArea.CLAW),
//                new TouchSensor(r, "TI", HardwareComponentArea.INTAKE),
//                new ColorSensor(r, "CC", HardwareComponentArea.CLAW),
//                new ColorSensor(r, "CI", HardwareComponentArea.INTAKE),
                new IMU(r, "IMU"),
                //new USBWebcam(r, "Webcam")
        });
    }

}
