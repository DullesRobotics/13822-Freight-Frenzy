package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumDriveTrain(this, false);
        waitForStart();

        robot.getLogger().usesDynamicData(true);
        robot.getLogger().setDynamicDataHeader("Robot Variables", false);

        robot.driveWithController(robot.ctrl1());
        robot.autoStrafeEncodedPID(5, new PID(0,0,0));

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
