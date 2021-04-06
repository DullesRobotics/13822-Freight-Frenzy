package org.firstinspires.ftc.teamcode.TestRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.*;

public class Configurator {

    public static Pose2d currentPosition = new Pose2d(0,0, Math.toRadians(0));

    /**
     * It's HIGHLY recommended every motor has the same motor configuration for autonomous driving
     * If not, individual motors might move incorrectly. If you have a mix-match of motors, don't use road runner.
     * @return The main hardware list for the robot
     */
    public static HardwareComponent[] getHardware(Robot robot){

        HardwareComponent[] driveTrainMotors = getDriveTrainMotors(robot);

        Motor m = new Motor(robot, "CLM", CLAW, true);
        m.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return(new HardwareComponent[]{
                driveTrainMotors[0],
                driveTrainMotors[1],
                driveTrainMotors[2],
                driveTrainMotors[3],
                new Motor(robot, "INM", INTAKE, false),
                new Motor(robot, "INM2", INTAKE, false),
                new Motor(robot, "SHM", SHOOTER, false),
                m,
                new Servo(robot, "CLS", CLAW),
                new Servo(robot, "CLS2", CLAW),
                new Servo(robot, "SHS", SHOOTER),
                new IMU(robot, "IMU"),
                new USBWebcam(robot, "Webcam")
        });
    }

    public static HardwareComponent[] getDriveTrainMotors(Robot robot){

        MotorConfiguration MC = new MotorConfiguration(
                MotorType.NEVEREST_ORBITAL,
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
