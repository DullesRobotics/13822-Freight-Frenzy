package org.firstinspires.ftc.teamcode.TestRobot.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;

@TeleOp
public class ShooterTest extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        IMU i = new IMU(robot, "IMU");
        robot.addHardware(
                new Motor(robot, "SHM", ComponentArea.SHOOTER, false),
                new Servo(robot, "SHS", ComponentArea.SHOOTER),
                i
        );

        i.startIMU();

        waitForStart();

        /* Robot functions */
        Functions.startShooter(robot, robot.ctrl1());

        while (opModeIsActive()) {
            i.updateIMU();
            robot.getLogger().putData("IMU Pitch", i.getPitch());
            robot.getLogger().putData("IMU Roll", i.getRoll());
            robot.getLogger().putData("IMU Yaw", i.getYaw());
            robot.getLogger().updateLog();
        }

        robot.stopAllThreads();
    }
}