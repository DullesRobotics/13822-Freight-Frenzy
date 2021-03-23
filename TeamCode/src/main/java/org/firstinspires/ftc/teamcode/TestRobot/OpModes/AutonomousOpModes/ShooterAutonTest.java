package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;

@Autonomous
public class ShooterAutonTest extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(
                new Motor(robot, "SHM", ComponentArea.SHOOTER, false),
                new Servo(robot, "SHS", ComponentArea.SHOOTER)
        );

        waitForStart();

        if (isStopRequested()) return;

        Functions.calibrateShooterServos(robot);

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        Functions.setShooterMotor(robot, true);

        robot.autonWait(2000);

        Functions.moveShooterServos(robot);

        robot.autonWait(2000);

        Functions.moveShooterServos(robot);

        robot.autonWait(2000);

        Functions.setShooterMotor(robot, false);

        robot.autonWait(10000);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
