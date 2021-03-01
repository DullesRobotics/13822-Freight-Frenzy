package org.firstinspires.ftc.teamcode.TestRobot.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.RobotRecorder;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.Functions;

@TeleOp
public class MainMode extends LinearOpMode {

    private MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this, /*HardwareConfigurator.getHardware(robot)*/ new HardwareComponent[]{});

        robot.getLogger().setDynamicDataHeader("Robot Variables");

        /* Add-on initializing */
        //UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        //robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, OpenCvInternalCamera.CameraDirection.BACK, OpenCvCameraRotation.UPRIGHT));
        robot.addOnManager().initAddOn(new RobotRecorder());

        waitForStart();

        /* Add-on starting */
        robot.addOnManager().startAddOn(AddOnType.ROBOT_RECORDER);

        /* Robot functions */
        robot.driveWithController(robot.ctrl1());
        Functions.startIntake(robot);
        Functions.startShooter(robot);

        while (opModeIsActive())
            robot.stopAllThreads();
    }
}
