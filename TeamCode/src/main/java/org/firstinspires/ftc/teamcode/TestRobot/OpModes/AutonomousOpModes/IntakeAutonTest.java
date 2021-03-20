package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.Functions;

@Autonomous
public class IntakeAutonTest extends LinearOpMode {

    StandardDriveTrain robot;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new StandardDriveTrain(this, new PID(0,0,0));
        robot.addHardware(Configurator.getHardware(robot));

        waitForStart();

        if (isStopRequested()) return;

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        Functions.setIntake(robot, true);

        robot.autonWait(1000);

        Functions.setIntake(robot, false);

        robot.autonWait(1000);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
