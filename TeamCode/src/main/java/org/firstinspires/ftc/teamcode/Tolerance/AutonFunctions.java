package org.firstinspires.ftc.teamcode.Tolerance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

public class AutonFunctions {
    private static volatile MecanumDriveTrain robot;
    private static volatile SampleMecanumDrive roadrunner;
    Motor m = robot.getMotor("CLM");
    public AutonFunctions(LinearOpMode op){
        roadrunner = new SampleMecanumDrive(op);
        robot = roadrunner.getDriveTrain();
    }

    public void downClaw(){

    }

    public void grab(){

    }

    public void upClaw(){

    }

    public void ungrab(){

    }

    public void shoot(){

    }

}
