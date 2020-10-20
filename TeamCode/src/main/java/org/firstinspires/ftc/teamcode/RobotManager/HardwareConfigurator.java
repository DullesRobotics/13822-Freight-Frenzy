package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor;
import org.firstinspires.ftc.teamcode.Libraries.IMUHandler;

public class HardwareConfigurator {

    public final static String ID_frontLeft = "FL", ID_frontRight = "FR", ID_backLeft = "BL", ID_backRight = "BR";

    /**
     * Add the hardware to the robot class
     * @param robot The robot this is configuring for
     * @param op The current op mode
     */
    protected static void configureHardware(Robot robot, LinearOpMode op){
        MotorConfiguration motorConfiguration = new MotorConfiguration(MotorConfiguration.MotorType.CORE_HEX_MOTOR, 3, true);
        Motor parentMotorLeft = new Motor(op, ID_frontLeft, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, true, true);
        Motor parentMotorRight = new Motor(op, ID_frontRight, HardwareComponentArea.DRIVE_TRAIN, motorConfiguration, true, false);

        /*
           Don't change the drive train motor IDs
         */
        robot.addHardware(
                parentMotorLeft, //left front motor
                parentMotorRight, //right front motor
                parentMotorLeft.clone(ID_backLeft), //left back motor
                parentMotorRight.clone(ID_backRight)); //right back motor

        robot.addHardware(
                new Servo(op, "CL", HardwareComponentArea.CLAW),
                new Servo(op, "IN", HardwareComponentArea.INTAKE),
                new TouchSensor(op, "TC", HardwareComponentArea.CLAW),
                new TouchSensor(op, "TI", HardwareComponentArea.INTAKE),
                new ColorSensor(op, "CC", HardwareComponentArea.CLAW),
                new ColorSensor(op, "CI", HardwareComponentArea.INTAKE),
                new IMUHandler(op, "IMU", robot));
    }

}
