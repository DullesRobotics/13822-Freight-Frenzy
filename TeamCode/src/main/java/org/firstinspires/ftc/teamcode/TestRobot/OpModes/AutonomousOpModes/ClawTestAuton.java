package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.SHOOTER;

@Autonomous
public class ClawTestAuton extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));
        waitForStart();

        int clawStartingPos = robot.getMotor("CLM").get().getCurrentPosition();

        if (isStopRequested()) return;

        Functions.setClawServos(robot, false);

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        Functions.setClawArmPosition(robot, true);
        Functions.setClawServos(robot, true);

        robot.autonWait(5000);

        Functions.setClawArmPosition(robot, false);

        robot.autonWait(3000);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
