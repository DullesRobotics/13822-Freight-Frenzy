package org.firstinspires.ftc.teamcode.Samurai.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.Samurai.Configurator;

//@TeleOp
public class StandardDriveTest extends LinearOpMode {

    private StandardDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new StandardDriveTrain(this);
        robot.addHardware(Configurator.getDriveTrainMotors(robot));

        waitForStart();

        robot.driveWithController(robot.ctrl1());

        while (opModeIsActive())
            robot.getLogger().updateLog();

        robot.stopAllThreads();
    }
}