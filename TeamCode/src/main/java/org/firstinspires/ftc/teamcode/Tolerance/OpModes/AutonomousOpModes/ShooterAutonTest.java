package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;

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
