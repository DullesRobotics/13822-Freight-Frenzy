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

import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_END_POS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_START_POS;

@Autonomous
public class ShooterAutonTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    private SampleMecanumDrive roadrunner;
    public static double startPower = 0.5, endPower = 1, diff = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        roadrunner = new SampleMecanumDrive(this);
        robot = roadrunner.getDriveTrain();

        waitForStart();

        while(opModeIsActive())
            Functions.setShooterMotor(robot, true);

        robot.stopAllThreads();
    }
}
