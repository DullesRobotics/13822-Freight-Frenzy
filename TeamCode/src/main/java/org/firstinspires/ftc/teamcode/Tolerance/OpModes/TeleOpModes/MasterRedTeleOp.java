package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.ControlCenterTeleOp;

public class MasterRedTeleOp extends LinearOpMode {
    private MecanumDriveTrain baseRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        baseRobot = new MecanumDriveTrain(this);
        baseRobot.addHardware(Configurator.getDriveTrainMotors(baseRobot));

        waitForStart();

        baseRobot.driveWithController(baseRobot.ctrl1());
        ControlCenterTeleOp.carouselSpin(baseRobot, baseRobot.ctrl1(), false);


        while (opModeIsActive())
            baseRobot.getLogger().updateLog();

        baseRobot.stopAllThreads();
    }
}
