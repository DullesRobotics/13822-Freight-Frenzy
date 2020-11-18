package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

@Autonomous
public class AutonMode extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        waitForStart();

        robot.getLogger().usesDynamicData(true);
        robot.getLogger().setDynamicDataHeader("Robot Variables");

        robot.autoStrafeEncoded(-50);
        //:D

        robot.stopAllThreads();
    }
}
