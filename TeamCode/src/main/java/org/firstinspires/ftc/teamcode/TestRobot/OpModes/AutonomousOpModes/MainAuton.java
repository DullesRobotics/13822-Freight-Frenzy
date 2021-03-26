package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_END_TICKS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_MID_TICKS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_PWR;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_END_POS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_START_POS;

public class MainAuton {

    private static volatile MecanumDriveTrain robot;
    private static volatile SampleMecanumDrive roadrunner;

    public static void start(LinearOpMode op, Pose2d startPose, Team team) throws InterruptedException {

        roadrunner = new SampleMecanumDrive(op);
        robot = roadrunner.getDriveTrain();

        int clawStartingPos = robot.getMotor("CLM").get().getCurrentPosition();

        robot.addThread(new Thread(() -> {
            while(op.opModeIsActive() && !op.isStopRequested())
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

        switch(amount){
            case ONE: zonePoint = ZONE_B; break;
            case FOUR: zonePoint = ZONE_C; break;
            case NONE:
            default: zonePoint = ZONE_A; break;
        }

        robot.autonWait(200);

        Trajectory wobbleOneTrajectory1 = roadrunner.trajectoryBuilder(startPose.plus(new Pose2d(0,0, Math.toRadians(-START_STACK_ANGLE))))
                .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(0)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(wobbleOneTrajectory1);

        Trajectory wobbleOneTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory1.end())
                .splineToLinearHeading(new Pose2d(zonePoint.y + FIRST_WOBBLE_OFFSET_Y, -zonePoint.x - FIRST_WOBBLE_OFFSET_X, Math.toRadians(180)), Math.toRadians(0))
                .build();
        
        roadrunner.followTrajectory(wobbleOneTrajectory2);

        robot.autonWait(200);

        Motor m = robot.getMotor("CLM");
        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);

        robot.autonWait(1000);

        Functions.setClawServos(robot, true);

        robot.autonWait(500);

        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);

        robot.autonWait(1000);

        m.get().setPower(0);

        /* trajectory to move to shooting position */
        Trajectory safeDistanceTrajectory1 = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end())
                .splineToConstantHeading(new Vector2d(
                        wobbleOneTrajectory2.end().getX() - WOBBLE_SAFE_CLAW_ARM_DISTANCE,
                        wobbleOneTrajectory2.end().getY()),
                        Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(safeDistanceTrajectory1);

        robot.autonWait(250);
        Functions.calibrateShooterServos(robot);
        Functions.setShooterMotor(robot, true);

        Trajectory safeDistanceTrajectory2 = roadrunner.trajectoryBuilder(safeDistanceTrajectory1.end())
                .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.y, -SHOOTING_POSITION_BLUE.x, Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(safeDistanceTrajectory2);

        robot.autonWait(250);

//        /* shoot 3 times */
//        for(int i = 0; i < 3; i++)
//            Functions.useShooterServos(robot);

        double currentPower = SHOOTER_POWER;

        Functions.setShooterMotor(robot, true, SHOOTER_POWER);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(500);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
        robot.autonWait(1000);

//        currentPower += POWER_TO_CHANGE;
//        Functions.setShooterMotor(robot, true, currentPower);
//        roadrunner.turn(Math.toRadians(ANGLE_TO_CHANGE));
        Functions.setShooterMotor(robot, true, SHOOTER_POWER);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(500);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
        robot.autonWait(1000);

//        currentPower += POWER_TO_CHANGE;
//        Functions.setShooterMotor(robot, true, currentPower);
//        roadrunner.turn(Math.toRadians(ANGLE_TO_CHANGE));
        Functions.setShooterMotor(robot, true, SHOOTER_POWER);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(500);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
        robot.autonWait(1000);
        Functions.useShooterServos(robot);

        robot.autonWait(250);

//
//        Trajectory secondWobbleGoalTrajectory = roadrunner.trajectoryBuilder(safeDistanceTrajectory.end())
//                .splineToLinearHeading(new Pose2d(BLUE_RETURN_POINT_2.y, -BLUE_RETURN_POINT_2.x, Math.toRadians(180)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(BLUE_RETURN_POINT_3.y, -BLUE_RETURN_POINT_3.x), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    Functions.setClawServos(robot, true);
//                    Functions.setClawArmPosition(robot, true, clawStartingPos);
//                })
//                .splineToConstantHeading(new Vector2d(BLUE_RETURN_POINT_3.y + INCHES_FORWARD_FOR_WOBBLE_PICKUP, -BLUE_RETURN_POINT_3.x), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    Functions.setClawServos(robot, false);
//                    robot.autonWait(250);
//                    Functions.setClawArmPosition(robot, false, clawStartingPos);
//                    robot.autonWait(250);
//                })
//                .splineToLinearHeading(new Pose2d(-zonePoint.y - SECOND_WOBBLE_OFFSET, zonePoint.x + SECOND_WOBBLE_OFFSET, Math.toRadians(0)), Math.toRadians(0))
//                .build();
//
//        roadrunner.followTrajectory(secondWobbleGoalTrajectory);
//
//        robot.autonWait(250);
//        Functions.setClawArmPosition(robot, true, clawStartingPos);
//        robot.autonWait(500);
//        Functions.setClawServos(robot, true);
//        robot.autonWait(250);
//
        double x = safeDistanceTrajectory2.end().getX(), y = safeDistanceTrajectory2.end().getY();
//
        Trajectory moveToLine = roadrunner.trajectoryBuilder(safeDistanceTrajectory2.end())
//                .splineToConstantHeading(new Vector2d(x - WOBBLE_SAFE_CLAW_ARM_DISTANCE, y), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(x, y - 6), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, y - 6), Math.toRadians(0))
//                .addDisplacementMarker(() -> {
//                    Functions.setClawArmPosition(robot, false, clawStartingPos);
//                    Functions.setClawServos(robot, false);
//                })
                .build();

        roadrunner.followTrajectory(moveToLine);

        robot.stopAllThreads();
    }

    public enum Team {
        RED, BLUE
    }
}
