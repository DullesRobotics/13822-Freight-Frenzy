package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;

@TeleOp
public class ShooterTest extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));



        waitForStart();

        /* Robot functions */
        Functions.startShooter(robot, robot.ctrl1());

        robot.stopAllThreads();
    }
}
