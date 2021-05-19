package org.firstinspires.ftc.teamcode.Tolerance.OpModes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.HighGoalDetectionPipeline;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.RingDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class Main extends LinearOpMode {

    private MecanumDriveTrain robot;
    public static OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));

        HighGoalDetectionPipeline goalPipeline = new HighGoalDetectionPipeline();
        EasyOpenCV easyOpenCV = new EasyOpenCV(goalPipeline, robot.getUSBWebcam(), rotation);
        robot.addOnManager().initAndStartAddOn(easyOpenCV);

        waitForStart();

        Functions.setWebcamServo(robot, false, easyOpenCV, null, goalPipeline);

        /* Robot functions */
        robot.driveWithController(robot.ctrl1());
        Functions.startIntake(robot, robot.ctrl2());
        Functions.startShooter(robot, robot.ctrl2());
        Functions.startClaw(robot, robot.ctrl2());
        Functions.toggleWebcam(robot, robot.ctrl1(), easyOpenCV, null, goalPipeline, false);

        while (opModeIsActive()) {
            //robot.getLogger().putData("PostEstimate", "(" + roadrunner.getPoseEstimate().getX() + ", " + roadrunner.getPoseEstimate().getY() + ") @ " + Math.toDegrees(roadrunner.getPoseEstimate().getHeading()) + "°");
            robot.getLogger().updateLog();
        }

        robot.stopAllThreads();
    }
}
