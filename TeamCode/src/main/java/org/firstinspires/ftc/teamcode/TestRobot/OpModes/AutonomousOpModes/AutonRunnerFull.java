package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.OPEN_CV_CAM_ROTATION;
import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.START_STACK_ANGLE;
import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.ZONE_A;
import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.ZONE_B;
import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.ZONE_C;

public class AutonRunnerFull {

    private static volatile MecanumDriveTrain robot;
    private static volatile SampleMecanumDrive roadrunner;

    public static void start(LinearOpMode op, Pose2d startPose, AutonRunner.Team team) throws InterruptedException {

        roadrunner = new SampleMecanumDrive(op);
        robot = roadrunner.getDriveTrain();

        robot.addThread(new Thread(() -> {
            while (op.opModeIsActive() && !op.isStopRequested())
                robot.getLogger().updateLog();
        }), true);

        roadrunner.setPoseEstimate(startPose);

        /* start OpenCV */
        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam(), OPEN_CV_CAM_ROTATION));

        Point zonePoint;

        op.waitForStart();

        if (op.isStopRequested()) return;

        Functions.setClawServos(robot, false);

        /* rotate towards start stack */
        roadrunner.turn(Math.toRadians(-START_STACK_ANGLE));

        /*
         * NONE - Zone A (lowest)
         * ONE - Zone B (middle)
         * FOUR - Zone C (top)
         */
        UltimateGoalPipeline.RingAmount amount = pipeline.getAmount();

        switch (amount) {
            case ONE:
                zonePoint = ZONE_B;
                break;
            case FOUR:
                zonePoint = ZONE_C;
                break;
            case NONE:
            default:
                zonePoint = ZONE_A;
                break;
        }

    }


    public enum Team {
        RED, BLUE
    }

}

