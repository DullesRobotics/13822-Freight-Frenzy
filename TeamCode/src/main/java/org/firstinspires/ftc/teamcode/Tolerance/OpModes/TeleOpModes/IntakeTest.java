package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;

@TeleOp
public class IntakeTest extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        robot.addHardware(new Motor(robot, "INM", ComponentArea.INTAKE, false));

        waitForStart();

        /* Robot functions */
        Functions.startIntake(robot, robot.ctrl1());

        while (opModeIsActive())
            robot.getLogger().updateLog();

        robot.stopAllThreads();
    }
}
