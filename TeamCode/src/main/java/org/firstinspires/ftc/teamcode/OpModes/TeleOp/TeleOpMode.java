package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotManager.DriveTrain;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        DriveTrain robot = new DriveTrain(this, false);
        waitForStart();

        robot.getLogger().usesDynamicData(true);
        robot.getLogger().setDynamicDataHeader("Robot Variables", false);

        robot.standardDriveWithController(robot.ctrl1());

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
