package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes.StartingPositions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tolerance.AutonRunner;

import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.STARTING_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.START_STACK_ANGLE;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonRunner.Team.RED;

@Autonomous
@Config
public class P_RedRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutonRunner.start(this, STARTING_RED_RIGHT, RED, AutonRunner.Side.RIGHT);

    }
}
