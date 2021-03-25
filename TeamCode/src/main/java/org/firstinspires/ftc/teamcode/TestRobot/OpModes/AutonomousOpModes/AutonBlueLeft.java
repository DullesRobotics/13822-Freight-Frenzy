package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.*;

@Autonomous
public class AutonBlueLeft extends LinearOpMode {

    private MecanumDriveTrain robot;
    private SampleMecanumDrive roadrunner;

    @Override
    public void runOpMode() throws InterruptedException {

        roadrunner = new SampleMecanumDrive(this);
        robot = roadrunner.getDriveTrain();

        robot.addThread(new Thread(() -> {
            while(opModeIsActive() && !isStopRequested())
                robot.getLogger().updateLog();
        }), true);

        Pose2d startPose = new Pose2d(STARTING_BLUE_LEFT.x, STARTING_BLUE_LEFT.y, Math.toRadians(0));
        roadrunner.setPoseEstimate(startPose);

        /* start OpenCV */
        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam(), OPEN_CV_CAM_ROTATION));

        Point zonePoint;

        waitForStart();

        if (isStopRequested()) return;

        robot.autonWait(250);

        /* rotate towards start stack */
        roadrunner.turn(-START_STACK_ANGLE);

        /* give time to analyze */
        robot.autonWait(500);

        /*
         * NONE - Zone A (lowest)
         * ONE - Zone B (middle)
         * FOUR - Zone C (top)
         */
        UltimateGoalPipeline.RingAmount amount = pipeline.getAmount();

        switch(amount){
            case ONE: zonePoint = ZONE_B; break;
            case FOUR: zonePoint = ZONE_C; break;
            case NONE:
            default: zonePoint = ZONE_A; break;
        }

        Trajectory wobbleOneTrajectory = roadrunner.trajectoryBuilder(startPose.plus(new Pose2d(0,0, Math.toRadians(-START_STACK_ANGLE))))
                .splineToLinearHeading(new Pose2d(zonePoint.x + FIRST_WOBBLE_OFFSET, zonePoint.y + FIRST_WOBBLE_OFFSET, Math.toRadians(0)), Math.toRadians(0))
                .build();
        
        roadrunner.followTrajectory(wobbleOneTrajectory);

        Functions.setClawArmPosition(robot, true);
        robot.autonWait(500);
        Functions.setClawServos(robot, true);
        robot.autonWait(250);

        /* trajectory to move to shooting position */
        Trajectory safeDistanceTrajectory = roadrunner.trajectoryBuilder(wobbleOneTrajectory.end())
                .splineToConstantHeading(new Vector2d(
                        wobbleOneTrajectory.end().getX(),
                        wobbleOneTrajectory.end().getY() - WOBBLE_SAFE_CLAW_ARM_DISTANCE),
                        Math.toRadians(0))
                .addDisplacementMarker(4, () -> {
                    Functions.setClawServos(robot, false);
                    robot.autonWait(250);
                    Functions.setClawArmPosition(robot, false);
                    Functions.calibrateShooterServos(robot);
                    Functions.setShooterMotor(robot, true);
                })
                .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.x, SHOOTING_POSITION_BLUE.y, Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(safeDistanceTrajectory);

        robot.autonWait(1000);

        /* shoot 3 times */
        for(int i = 0; i < 3; i++)
            Functions.useShooterServos(robot);

        robot.autonWait(250);

        Trajectory secondWobbleGoalTrajectory = roadrunner.trajectoryBuilder(safeDistanceTrajectory.end())
                .splineToLinearHeading(new Pose2d(BLUE_RETURN_POINT_2.x, BLUE_RETURN_POINT_2.y, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(BLUE_RETURN_POINT_3.x, BLUE_RETURN_POINT_3.y), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    Functions.setClawServos(robot, true);
                    Functions.setClawArmPosition(robot, true);
                })
                .splineToConstantHeading(new Vector2d(BLUE_RETURN_POINT_3.x, BLUE_RETURN_POINT_3.y + INCHES_FORWARD_FOR_WOBBLE_PICKUP), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    Functions.setClawServos(robot, false);
                    robot.autonWait(250);
                    Functions.setClawArmPosition(robot, false);
                    robot.autonWait(250);
                })
                .splineToLinearHeading(new Pose2d(zonePoint.x + SECOND_WOBBLE_OFFSET, zonePoint.y + SECOND_WOBBLE_OFFSET, Math.toRadians(0)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(secondWobbleGoalTrajectory);

        robot.autonWait(250);
        Functions.setClawArmPosition(robot, true);
        robot.autonWait(500);
        Functions.setClawServos(robot, true);
        robot.autonWait(250);

        double x = secondWobbleGoalTrajectory.end().getX(), y = secondWobbleGoalTrajectory.end().getY();

        Trajectory moveToLine = roadrunner.trajectoryBuilder(secondWobbleGoalTrajectory.end())
                .splineToConstantHeading(new Vector2d(x, y - WOBBLE_SAFE_CLAW_ARM_DISTANCE), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(x + 6, y), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(x + 6, LAUNCH_LINE_Y_COORDINATE), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    Functions.setClawArmPosition(robot, false);
                    Functions.setClawServos(robot, false);
                })
                .build();

        roadrunner.followTrajectory(moveToLine);

        robot.stopAllThreads();
    }
}
