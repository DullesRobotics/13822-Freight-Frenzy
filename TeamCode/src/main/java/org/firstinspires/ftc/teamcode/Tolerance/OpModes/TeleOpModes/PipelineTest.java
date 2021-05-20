package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.HighGoalDetectionPipeline;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.RingDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
public class PipelineTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    public static OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));

        RingDetectionPipeline ringPipeline = new RingDetectionPipeline();
        HighGoalDetectionPipeline goalPipeline = new HighGoalDetectionPipeline();
        EasyOpenCV easyOpenCV = new EasyOpenCV(ringPipeline, robot.getUSBWebcam(), rotation);
        robot.addOnManager().initAndStartAddOn(easyOpenCV);

        waitForStart();

        Functions.toggleWebcam(robot, robot.ctrl1(), easyOpenCV, ringPipeline, goalPipeline, true);
        //Functions.autoAim(robot, goalPipeline, EasyOpenCV.VIEWPORT_WIDTH);

        /* Robot functions */

        while (opModeIsActive()) {
            robot.getLogger().putData("opencv analysis", ringPipeline.getAnalysis());
            robot.getLogger().putData("ring amount", ringPipeline.getAmount());
            robot.getLogger().updateLog();
        }

        robot.stopAllThreads();
    }
}
