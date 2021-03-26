package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.StartingPositions;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.MainAuton;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes.MainAuton.Team.*;

@Autonomous
public class BlueLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(STARTING_BLUE_LEFT.y, -STARTING_BLUE_LEFT.x, Math.toRadians(0));
        MainAuton.start(this, startPose, BLUE);

    }
}
