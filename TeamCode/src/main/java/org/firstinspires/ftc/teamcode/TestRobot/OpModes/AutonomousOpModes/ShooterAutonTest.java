package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;

import java.util.logging.Level;

@Autonomous
public class ShooterAutonTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    private SampleMecanumDrive roadrunner;
    public static double angleDiff = -5, startingAngle = 35, endingAngle = -35;

    @Override
    public void runOpMode() throws InterruptedException {

        double currentAngle = startingAngle;

        roadrunner = new SampleMecanumDrive(this);
        robot = roadrunner.getDriveTrain();

        waitForStart();

        roadrunner.turn(Math.toRadians(startingAngle));
        Functions.setShooterMotor(robot, true);


        while(currentAngle > endingAngle - angleDiff - 1) {
            robot.autonWait(2000);
            for(int i = 0; i < 3; i++)
                Functions.useShooterServos(robot);
            currentAngle += angleDiff;
            roadrunner.turn(Math.toRadians(currentAngle));
            robot.getLogger().log(Level.INFO, "Angle Testing: " + currentAngle + " degrees");
            robot.getLogger().updateLog();
        }

        Functions.setShooterMotor(robot, false);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
