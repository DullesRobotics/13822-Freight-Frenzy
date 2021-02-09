package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOns;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumDriveTrain(this);
        robot.addOnManager().initAddOn(AddOns.VUFORIA, false, VuforiaLocalizer.CameraDirection.BACK, false);
        robot.getLogger().setDynamicDataHeader("Robot Variables");

        waitForStart();

        robot.addOnManager().initAddOn(AddOns.ROBOT_RECORDER, true);
        robot.addOnManager().getAddOn(AddOns.VUFORIA).start();


        robot.driveWithController(robot.ctrl1());

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
