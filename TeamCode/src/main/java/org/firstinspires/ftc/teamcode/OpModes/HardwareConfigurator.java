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
        Motor parentMotorLeft = new Motor(r, ID_frontLeft, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, true);
        Motor parentMotorRight = new Motor(r, ID_frontRight, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, true);

        r.addHardware(
                parentMotorLeft, //left front motor
                parentMotorRight, //right front motor
                parentMotorLeft.clone(ID_backLeft), //left back motor
                parentMotorRight.clone(ID_backRight)); //right back motor

        r.addHardware(
                new Servo(r, "CL", HardwareComponentArea.CLAW),
                new Servo(r, "IN", HardwareComponentArea.INTAKE),
                new TouchSensor(r, "TC", HardwareComponentArea.CLAW),
                new TouchSensor(r, "TI", HardwareComponentArea.INTAKE),
                new ColorSensor(r, "CC", HardwareComponentArea.CLAW),
                new ColorSensor(r, "CI", HardwareComponentArea.INTAKE),
                new IMU(r, "IMU"));
    }

}
