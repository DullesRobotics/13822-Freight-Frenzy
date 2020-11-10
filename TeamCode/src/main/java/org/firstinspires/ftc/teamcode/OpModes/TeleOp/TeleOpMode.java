package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.OpModes.HardwareConfigurator;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDriveTrain robot = new MecanumDriveTrain(this, false, HardwareConfigurator.ID_frontLeft, HardwareConfigurator.ID_frontRight, HardwareConfigurator.ID_backLeft, HardwareConfigurator.ID_backRight);
        waitForStart();

        robot.getLogger().usesDynamicData(true);
        robot.getLogger().setDynamicDataHeader("Robot Variables", false);

        robot.driveWithController(robot.ctrl1());
        robot.autoStrafeEncodedPID(5, new PID(0,0,0));

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
