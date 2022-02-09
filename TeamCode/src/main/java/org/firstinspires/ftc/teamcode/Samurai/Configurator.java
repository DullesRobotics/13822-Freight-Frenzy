package org.firstinspires.ftc.teamcode.Samurai;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.CAROUSEL;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
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

        HardwareComponent[] driveTrainMotors = getDriveTrainMotors(robot);
        Motor carouselMotor = new Motor(robot, "CAR" , CAROUSEL ,false );

        return(new HardwareComponent[]{
                driveTrainMotors[0],
                driveTrainMotors[1],
                driveTrainMotors[2],
                driveTrainMotors[3],
                //new Motor(robot, "COW", CENTER_OMNI, true),
                new IMU(robot, "IMU"),
                new Motor(robot, "INT", ComponentArea.INTAKE, false),
                //new USBWebcam(robot, "Webcam"),
                carouselMotor
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
