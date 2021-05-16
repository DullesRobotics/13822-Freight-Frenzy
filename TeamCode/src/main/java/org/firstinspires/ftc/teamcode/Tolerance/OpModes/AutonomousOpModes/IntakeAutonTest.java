package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;

@Autonomous
public class IntakeAutonTest extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(new Motor(robot, "INM", ComponentArea.INTAKE, false));

        waitForStart();

        if (isStopRequested()) return;

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        Functions.setIntake(robot, true, true);

        robot.autonWait(1000);

        Functions.setIntake(robot, false, true);

        robot.autonWait(1000);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
