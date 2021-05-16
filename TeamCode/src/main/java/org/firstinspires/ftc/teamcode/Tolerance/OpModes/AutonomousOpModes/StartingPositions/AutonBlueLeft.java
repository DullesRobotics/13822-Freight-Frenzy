package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes.StartingPositions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes.AutonRunner;

import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes.AutonRunner.Team.*;

@Autonomous
@Config
public class AutonBlueLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(STARTING_BLUE_LEFT.y, -STARTING_BLUE_LEFT.x, Math.toRadians(-START_STACK_ANGLE));
        AutonRunner.start(this, startPose, BLUE);

    }
}
