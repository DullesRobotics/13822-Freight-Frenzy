package org.firstinspires.ftc.teamcode.TestRobot.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.firstinspires.ftc.teamcode.TestRobot.HardwareConfigurator;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumDriveTrain(this, /*HardwareConfigurator.getHardware(robot)*/ new HardwareComponent[]{});

        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();

        robot.addOnManager().initAddOn(new EasyOpenCV(pipeline, OpenCvInternalCamera.CameraDirection.BACK, OpenCvCameraRotation.UPRIGHT));
        //robot.addOnManager().initAddOn(new RobotRecorder());
        robot.getLogger().setDynamicDataHeader("Robot Variables");

        waitForStart();

        robot.addOnManager().startAddOn(AddOnType.EASY_OPEN_CV);
        //robot.addOnManager().startAddOn(AddOnType.ROBOT_RECORDER);

        // pipeline.getAmount(); //amount of rings

        //robot.driveWithController(robot.ctrl1());

        while (opModeIsActive())
        robot.stopAllThreads();
    }
}
