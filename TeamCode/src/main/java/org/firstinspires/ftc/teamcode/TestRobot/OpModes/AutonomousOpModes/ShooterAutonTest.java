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

        double currentPower = startPower;

        waitForStart();

        while(currentPower < endPower + diff) {
            Functions.setShooterMotor(robot, true, currentPower);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_END_POS);
            robot.autonWait(500);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_START_POS);
            robot.autonWait(1000);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_END_POS);
            robot.autonWait(500);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_START_POS);
            robot.autonWait(1000);
            Functions.useShooterServos(robot);
            currentPower += diff;
        }

        Functions.setShooterMotor(robot, false);

        Functions.setShooterMotor(robot, false);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
