package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.StartingPositions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.AutonRunner;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.AutonRunnerAdjusted;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.AutonRunner.Team.*;

@Autonomous
@Config
public class AutonBlueLeft extends LinearOpMode {

    public static boolean adjusted = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = adjusted ? new Pose2d(-57, 48, Math.toRadians(0)) : new Pose2d(STARTING_BLUE_LEFT.y, -STARTING_BLUE_LEFT.x, Math.toRadians(0));
        if(adjusted)
                AutonRunnerAdjusted.start(this, startPose);
            else
                AutonRunner.start(this, startPose, BLUE);

    }
}
