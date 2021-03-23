package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Hardware.ComponentArea.SHOOTER;

@Autonomous
@Config
public class WobbleGoalAtLeft extends LinearOpMode {

    private MecanumDriveTrain robot;
    private SampleMecanumDrive roadrunner;
    public static OpenCvCameraRotation openCvCameraRotation = OpenCvCameraRotation.UPRIGHT;
    public static Point startingCoordinate = new Point(24, -60);
    public static double startingDegrees = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        roadrunner = new SampleMecanumDrive(this);
        robot = roadrunner.getDriveTrain();

        Pose2d startPose = new Pose2d(startingCoordinate.x, startingCoordinate.y, Math.toRadians(startingDegrees));

        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam(), openCvCameraRotation));

        waitForStart();

        if (isStopRequested()) return;

        robot.autonWait(1000);

        /*
         * NONE - Zone A (lowest)
         * ONE - Zone B (middle)
         * FOUR - Zone C (top)
         */
        UltimateGoalPipeline.RingAmount amount = pipeline.getAmount();

        switch(amount){
            case NONE:
                break;
            case ONE:
                break;
            case FOUR:
                break;
        }


        while (!isStopRequested() && opModeIsActive())
            robot.getLogger().updateLog();

        robot.stopAllThreads();
    }
}
