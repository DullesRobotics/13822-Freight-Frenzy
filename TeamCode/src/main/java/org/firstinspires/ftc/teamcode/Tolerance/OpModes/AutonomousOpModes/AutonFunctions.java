package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

public class AutonFunctions {
    private static volatile MecanumDriveTrain mainFrame;
    private static volatile SampleMecanumDrive roadRunner;
    private static volatile TeamColor team = TeamColor.blue;

    public static void start(LinearOpMode Op, TeamColor team, Direction position){

    }
    public enum TeamColor{
        red,blue;
    }
    public enum Direction{
        left,right;
    }
}
