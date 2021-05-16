package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.UltimateGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Config
public class OpenCVTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    public static OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        robot.addHardware(new USBWebcam(robot, "Webcam"));

        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        EasyOpenCV easyOpenCV = new EasyOpenCV(pipeline, robot.getUSBWebcam(), rotation);
        robot.addOnManager().initAndStartAddOn(easyOpenCV);

        waitForStart();

        /* Robot functions */

        while (opModeIsActive()) {
            robot.getLogger().putData("opencv analysis", pipeline.getAnalysis());
            robot.getLogger().putData("ring amount", pipeline.getAmount());
            robot.getLogger().updateLog();
        }

        robot.stopAllThreads();
    }
}
