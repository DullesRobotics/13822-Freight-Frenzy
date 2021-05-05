package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;

import java.util.logging.Level;

import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_END_POS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_START_POS;

@Autonomous
@Config
public class ShooterAutonTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    public static double shooterPower = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));

        waitForStart();

        while(opModeIsActive()) {
            Functions.setShooterMotor(robot, true);
        }

        robot.stopAllThreads();
    }
}
