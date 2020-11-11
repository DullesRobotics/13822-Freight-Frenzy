package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

public class HardwareConfigurator {

    public final static String ID_frontLeft = "FL", ID_frontRight = "FR", ID_backLeft = "BL", ID_backRight = "BR";

    /**
     * Add the hardware to the robot class
     * @param r The robot this is configuring for
     */
    public static void configureHardware(Robot r){
        MotorConfiguration motorConfiguration = new MotorConfiguration(MotorType.CORE_HEX_MOTOR, 3, true, true);
        Motor motorFrontLeft = new Motor(r, ID_frontLeft, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, false),
        motorFrontRight = new Motor(r, ID_frontRight, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, true),
        motorBackLeft = motorFrontLeft.clone(ID_backLeft),
        motorBackRight = motorFrontRight.clone(ID_backRight);

        //front left & back right are strafe opposite
        motorFrontLeft.setStrafeOpposite(true);
        motorBackRight.setStrafeOpposite(true);

        r.addHardware(
                motorFrontLeft, //left front motor
                motorFrontRight, //right front motor
                motorBackLeft, //left back motor
                motorBackRight, //right back motor
                new Servo(r, "CL", HardwareComponentArea.CLAW),
                new Servo(r, "IN", HardwareComponentArea.INTAKE),
                new TouchSensor(r, "TC", HardwareComponentArea.CLAW),
                new TouchSensor(r, "TI", HardwareComponentArea.INTAKE),
                new ColorSensor(r, "CC", HardwareComponentArea.CLAW),
                new ColorSensor(r, "CI", HardwareComponentArea.INTAKE),
                new IMU(r, "IMU"));
    }

}
