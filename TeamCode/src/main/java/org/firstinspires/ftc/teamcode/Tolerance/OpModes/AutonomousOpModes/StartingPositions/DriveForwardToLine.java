package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes.StartingPositions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tolerance.AutonRunner;

import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.STARTING_BLUE_LEFT;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.STARTING_RED_LEFT;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonRunner.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonRunner.Team.DRIVE_FORWARD;

@Autonomous
@Config
public class DriveForwardToLine extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutonRunner.start(this, STARTING_RED_LEFT, DRIVE_FORWARD, AutonRunner.Side.LEFT);

    }
}
