package org.firstinspires.ftc.teamcode.Tolerance;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.*;

public class Configurator {

    /**
     * It's HIGHLY recommended every motor has the same motor configuration for autonomous driving
     * If not, individual motors might move incorrectly. If you have a mix-match of motors, don't use road runner.
     * @return The main hardware list for the robot
     */
    public static HardwareComponent[] getHardware(Robot robot){

        HardwareComponent[] driveTrainMotors = getDriveTrainMotors(robot);

        return(new HardwareComponent[]{
                driveTrainMotors[0],
                driveTrainMotors[1],
                driveTrainMotors[2],
                driveTrainMotors[3],
                new IMU(robot, "IMU"),
                new USBWebcam(robot, "Webcam"),
        });
    }

    public static HardwareComponent[] getDriveTrainMotors(Robot robot){

        MotorConfiguration MC = new MotorConfiguration(
                MotorType.GO_BUILD_A_YELLOW_JACKET,
                true,
                2.9528,
                1);

        DrivetrainMotor motorFrontLeft = new DrivetrainMotor(robot, "FLM", MC, true, MotorType.DrivetrainPosition.FLM);
        DrivetrainMotor motorFrontRight = new DrivetrainMotor(robot, "FRM", MC, true, MotorType.DrivetrainPosition.FRM);
        DrivetrainMotor motorBackLeft = new DrivetrainMotor(robot, "BLM", MC, true, MotorType.DrivetrainPosition.BLM);
        DrivetrainMotor motorBackRight = new DrivetrainMotor(robot, "BRM", MC, true, MotorType.DrivetrainPosition.BRM);

        motorFrontLeft.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return(new HardwareComponent[]{
                motorFrontLeft,
                motorFrontRight,
                motorBackLeft,
                motorBackRight
        });
    }

}
