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
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.Functions;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.firstinspires.ftc.teamcode.TestRobot.PoseStorage;

import static org.firstinspires.ftc.teamcode.TestRobot.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_END_TICKS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_MID_TICKS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.CLAW_MOTOR_PWR;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_END_POS;
import static org.firstinspires.ftc.teamcode.TestRobot.Functions.SHOOTER_SERVO_START_POS;

public class AutonRunner {

    private static volatile MecanumDriveTrain robot;
    private static volatile SampleMecanumDrive roadrunner;

    public static void start(LinearOpMode op, Pose2d startPose, Team team) throws InterruptedException {

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
        int zone = 4;

        op.waitForStart();

        if (op.isStopRequested()) return;

        Functions.setClawServos(robot, false);

        /* rotate towards start stack */
//        roadrunner.turn(Math.toRadians(-START_STACK_ANGLE));

        /*
         * NONE - Zone A (lowest)
         * ONE - Zone B (middle)
         * FOUR - Zone C (top)
         */
        UltimateGoalPipeline.RingAmount amount = pipeline.getAmount();

        switch (amount) {
            case ONE:
                zonePoint = ZONE_B;
                zone = 1;
                break;
            case FOUR:
                zonePoint = ZONE_C;
                zone = 4;
                break;
            case NONE:
            default:
                zonePoint = ZONE_A;
                zone = 0;
                break;
        }

        int angle = 90;

        if((amount == UltimateGoalPipeline.RingAmount.NONE) || (amount == UltimateGoalPipeline.RingAmount.FOUR))
            angle = -90;


        Trajectory wobbleOneTrajectory2 = roadrunner.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(zonePoint.y + 10, -zonePoint.x, Math.toRadians(angle)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(wobbleOneTrajectory2);

        Motor m = robot.getMotor("CLM");
        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS+200);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);
        robot.autonWait(1000);

        m.get().setPower(0);

        Functions.setClawServos(robot, true);

        robot.autonWait(750);

        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);

        robot.autonWait(600);

        m.get().setPower(0);

        robot.autonWait(100);

//        Trajectory Middle = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end())
//                .splineToLinearHeading(new Pose2d(STARTING_BLUE_RIGHT.y+ ( zone == 0 ? 22 : 26), -STARTING_BLUE_RIGHT.x + (zone == 0 ? -2 : -6), Math.toRadians(0)),Math.toRadians(0))
//                .build();
//
//        roadrunner.followTrajectory(Middle);
//
//        m.get().setPower(0);
//        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
//        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS+200);
//        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m.get().setPower(CLAW_MOTOR_PWR);
//        robot.autonWait(1000);
//
//        m.get().setPower(0);
//
//        Functions.setClawServos(robot, false);
//
//        robot.autonWait(750);
//
//        m.get().setPower(0);
//        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
//        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
//        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m.get().setPower(CLAW_MOTOR_PWR);
//
//        robot.autonWait(600);
//
//        m.get().setPower(0);
//
//        robot.autonWait(100);
//
//
//        Trajectory wobble2 = roadrunner.trajectoryBuilder(Middle.end())
//                .splineToLinearHeading(new Pose2d(zonePoint.y + 10, -zonePoint.x, Math.toRadians(angle)), Math.toRadians(0))
//                .build();
//
//        roadrunner.followTrajectory(wobble2);
//
//
//        m.get().setPower(0);
//        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
//        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS+200);
//        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m.get().setPower(CLAW_MOTOR_PWR);
//        robot.autonWait(1000);
//
//        m.get().setPower(0);
//
//        Functions.setClawServos(robot, true);
//
//        robot.autonWait(750);
//
//        m.get().setPower(0);
//        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
//        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
//        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m.get().setPower(CLAW_MOTOR_PWR);
//
//        robot.autonWait(600);
//
//        m.get().setPower(0);
//
//        robot.autonWait(100);


        Trajectory safeDistanceTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end()  /*wobble2.end()*/)
                .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.y+2, -SHOOTING_POSITION_BLUE.x + (zone == 0 || zone == 4 ? -5 : -2), Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(safeDistanceTrajectory2);
        double x = safeDistanceTrajectory2.end().getX(), y = safeDistanceTrajectory2.end().getY();

        Functions.calibrateShooterServos(robot);
        roadrunner.turn(Math.toRadians(5));
        Functions.setShooterMotor(robot, true);
        robot.autonWait(1700);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);

        roadrunner.turn(Math.toRadians(-5));
        Functions.setShooterMotor(robot, true);
        robot.autonWait(1750);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);

        roadrunner.turn(Math.toRadians(-10));
        Functions.setShooterMotor(robot, true);
        robot.autonWait(1750);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);

        Trajectory moveToLine = roadrunner.trajectoryBuilder(safeDistanceTrajectory2.end().plus(new Pose2d(0,0,Math.toRadians(-15))))
                .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, y - 4), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(moveToLine);


        robot.stopAllThreads();
        PoseStorage.currentPose = roadrunner.getPoseEstimate();

    }


    public enum Team {
        RED, BLUE
    }
}

