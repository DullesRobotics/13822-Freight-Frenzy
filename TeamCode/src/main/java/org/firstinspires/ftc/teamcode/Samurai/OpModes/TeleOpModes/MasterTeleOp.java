package org.firstinspires.ftc.teamcode.Samurai.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Samurai.Configurator;
import org.firstinspires.ftc.teamcode.Samurai.Functions;

@TeleOp
public class MasterTeleOp extends LinearOpMode {
    private MecanumDriveTrain baseRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        baseRobot = new MecanumDriveTrain(this);
        baseRobot.addHardware(Configurator.getHardware(baseRobot));

        waitForStart();

        baseRobot.driveWithController(baseRobot.ctrl1());
        Functions.carouselSpin(baseRobot, baseRobot.ctrl2());
        Functions.intakeInOut(baseRobot, baseRobot.ctrl2());
        Functions.intakeUpDown(baseRobot, baseRobot.ctrl2());

        while (opModeIsActive())
            baseRobot.getLogger().updateLog();

        baseRobot.stopAllThreads();
    }
}
