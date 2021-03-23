package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.ViewerParameters;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.logging.Level;

@Autonomous
@Config
public class TrajectoryTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    private SampleMecanumDrive roadrunner;
    public static Point startingCoordinate = new Point(24, -60);
    public static double startingDegrees = 0;
    public static double xMoveInches = 10, yMoveInches = 20, degreesToTurn = 90;

    @Override
    public void runOpMode() throws InterruptedException {

        roadrunner = new SampleMecanumDrive(this);
        robot = roadrunner.getDriveTrain();

        Pose2d startPose = new Pose2d(startingCoordinate.x, startingCoordinate.y, Math.toRadians(startingDegrees));
        Trajectory trajectory = roadrunner.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(
                        startingCoordinate.x + xMoveInches,
                        startingCoordinate.y + yMoveInches,
                        Math.toRadians(startingDegrees)),
                        Math.toRadians(startingDegrees + degreesToTurn))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        try {
            roadrunner.followTrajectory(trajectory);
        } catch (PathContinuityViolationException e) {
            robot.getLogger().log(Level.SEVERE, e.getStackTrace().toString());
            robot.stopAllThreads();
            requestOpModeStop();
        }

        while (!isStopRequested() && opModeIsActive())
            robot.getLogger().updateLog();

        robot.stopAllThreads();
    }
}
