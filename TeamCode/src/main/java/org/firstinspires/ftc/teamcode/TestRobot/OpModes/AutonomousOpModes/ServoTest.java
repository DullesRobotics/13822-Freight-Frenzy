package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.Functions;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.SHOOTER;

@Autonomous
@Config
public class ServoTest extends LinearOpMode {

    StandardDriveTrain robot;
    SampleMecanumDrive drive;
    public static com.qualcomm.robotcore.hardware.Servo.Direction servoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new StandardDriveTrain(this, new PID(0,0,0));
        Servo s = new Servo(robot, "SHS", SHOOTER);
        robot.addHardware(s);

        waitForStart();

        if (isStopRequested()) return;

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        s.get().scaleRange(0,1);
        s.get().setDirection(servoDirection);
        s.get().setPosition(0);

        robot.autonWait(2000);

        s.get().setPosition(0.25);

        robot.autonWait(2000);

        s.get().setPosition(0.5);

        robot.autonWait(2000);

        s.get().setPosition(0.75);

        robot.autonWait(2000);

        s.get().setPosition(1);

        robot.autonWait(2000);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
