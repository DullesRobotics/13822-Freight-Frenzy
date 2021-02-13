package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.OpenCVPipelines.UltimateGoalPipeline;
import org.firstinspires.ftc.teamcode.OpModes.HardwareConfigurator;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumDriveTrain(this, HardwareConfigurator.getHardware(robot));

        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();

        robot.addOnManager().initAddOn(new EasyOpenCV(pipeline, OpenCvInternalCamera.CameraDirection.BACK, OpenCvCameraRotation.UPRIGHT));
        //robot.addOnManager().initAddOn(new RobotRecorder());
        robot.getLogger().setDynamicDataHeader("Robot Variables");

        waitForStart();

        robot.addOnManager().startAddOn(AddOnType.EASY_OPEN_CV);
        //robot.addOnManager().startAddOn(AddOnType.ROBOT_RECORDER);

        pipeline.getAmount(); //amount of rings

        //robot.driveWithController(robot.ctrl1());

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
