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

        robot.autonWait(200);

        Trajectory wobbleOneTrajectory1 = roadrunner.trajectoryBuilder(startPose.plus(new Pose2d(0, 0, Math.toRadians(-START_STACK_ANGLE))))
                .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(0)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(wobbleOneTrajectory1);

        Trajectory wobbleOneTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory1.end())
                .splineToLinearHeading(new Pose2d(zonePoint.y + 10, -zonePoint.x, Math.toRadians(180)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(wobbleOneTrajectory2);

        robot.autonWait(200);

        Motor m = robot.getMotor("CLM");
        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS);
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

        robot.autonWait(800);

        m.get().setPower(0);

        if (amount == UltimateGoalPipeline.RingAmount.NONE) {

            Trajectory wobbleTwoTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end())
                    .splineToLinearHeading(new Pose2d(-42.5+2, 33.5, Math.toRadians(90)), Math.toRadians(0))
                    .build();

            roadrunner.followTrajectory(wobbleTwoTrajectory2);

            m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS + 400);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);
        robot.autonWait(2000);
        Functions.setClawServos(robot, true);

        m.get().setPower(0);
        robot.autonWait(750);

        Functions.setClawServos(robot, false);

        robot.autonWait(500);

        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);

        robot.autonWait(800);

        m.get().setPower(0);


            Trajectory wobbleTwoTrajectoryMiddle = roadrunner.trajectoryBuilder(wobbleTwoTrajectory2.end())
                    .splineToLinearHeading(new Pose2d(zonePoint.y-1.5, -zonePoint.x, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            roadrunner.followTrajectory(wobbleTwoTrajectoryMiddle);

            m.get().setPower(0);
            m.get().setDirection(DcMotorSimple.Direction.FORWARD);
            m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS + 400);
            m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.get().setPower(CLAW_MOTOR_PWR);
            robot.autonWait(2000);

            m.get().setPower(0);

            Functions.setClawServos(robot, true);

            robot.autonWait(750);

            m.get().setPower(0);
            m.get().setDirection(DcMotorSimple.Direction.REVERSE);
            m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
            m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.get().setPower(CLAW_MOTOR_PWR);

            robot.autonWait(800);

            m.get().setPower(0);


            Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 100);

            Trajectory safeDistanceTrajectory2 = roadrunner.trajectoryBuilder(wobbleTwoTrajectoryMiddle.end())
                    .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.y+2, -SHOOTING_POSITION_BLUE.x + 2.5, Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                    .build();

            roadrunner.followTrajectory(safeDistanceTrajectory2);

            robot.autonWait(250);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_END_POS);
            robot.autonWait(500);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_START_POS);
            Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
            robot.autonWait(1250);

            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_END_POS);
            robot.autonWait(250);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_START_POS);
            Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
            robot.autonWait(1250);

            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_END_POS);
            robot.autonWait(250);
            for(Servo s : robot.getServos(ComponentArea.SHOOTER))
                s.get().setPosition(SHOOTER_SERVO_START_POS);

            robot.autonWait(250);

            double x = safeDistanceTrajectory2.end().getX(), y = safeDistanceTrajectory2.end().getY();

            Trajectory moveToLine = roadrunner.trajectoryBuilder(safeDistanceTrajectory2.end())
                    .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, y + 6), Math.toRadians(0))
                    .build();

            roadrunner.followTrajectory(moveToLine);

        }


//        robot.autonWait(1000);
//        Functions.calibrateShooterServos(robot);
//        Functions.setShooterMotor(robot, true, SHOOTER_POWER);
//
        else {
            Functions.setShooterMotor(robot, true , Functions.SHOOTER_SPEED - 150);
            Trajectory safeDistanceTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end())
                    .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.y+2, -SHOOTING_POSITION_BLUE.x+2, Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                    .build();

        roadrunner.followTrajectory(safeDistanceTrajectory2);

        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
        Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
        robot.autonWait(1000);

        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
        Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
        robot.autonWait(1000);

        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(500);
        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);

        robot.autonWait(250);

        double x = safeDistanceTrajectory2.end().getX(), y = safeDistanceTrajectory2.end().getY();

        Trajectory moveToLine = roadrunner.trajectoryBuilder(safeDistanceTrajectory2.end())
                .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, y - 6), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(moveToLine);
    }

        robot.stopAllThreads();
    }

    public enum Team {
        RED, BLUE
    }
}

