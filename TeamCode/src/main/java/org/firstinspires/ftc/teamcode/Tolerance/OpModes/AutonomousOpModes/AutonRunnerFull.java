package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.UltimateGoalPipeline;
import org.firstinspires.ftc.teamcode.Tolerance.PoseStorage;

import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_END_TICKS;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_MID_TICKS;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_PWR;

public class AutonRunnerFull {

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

        roadrunner.turn(Math.toRadians(START_STACK_ANGLE));

        int angle = 90;

        if((amount == UltimateGoalPipeline.RingAmount.NONE) || (amount == UltimateGoalPipeline.RingAmount.FOUR))
            angle = -90;

        Trajectory moveForward = roadrunner.trajectoryBuilder(new Pose2d(STARTING_BLUE_LEFT.y, -STARTING_BLUE_LEFT.x,0))
                .back(6)
                .build();


        Trajectory wobbleOneTrajectory2 = roadrunner.trajectoryBuilder(moveForward.end())
                .splineToLinearHeading(new Pose2d(zonePoint.y + 10, -zonePoint.x, Math.toRadians(angle)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(wobbleOneTrajectory2);

        robot.autonWait(200);

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
//
////        robot.autonWait(1000);
////        Functions.calibrateShooterServos(robot);
////        Functions.setShooterMotor(robot, true, SHOOTER_POWER);
////
////          Functions.setShooterMotor(robot, true , Functions.SHOOTER_SPEED - 150);
        Trajectory safeDistanceTrajectory2 = roadrunner.trajectoryBuilder(wobbleOneTrajectory2.end())
                .splineToLinearHeading(new Pose2d(SHOOTING_POSITION_BLUE.y+2, -SHOOTING_POSITION_BLUE.x+2, Math.toRadians(SHOOTING_ANGLE_BLUE)), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(safeDistanceTrajectory2);
        robot.autonWait(1000);


//
////        roadrunner.turn(ANGLE);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_END_POS);
////        robot.autonWait(250);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_START_POS);
////        Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
////        robot.autonWait(1000);
//
////        roadrunner.turn(ANGLE);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_END_POS);
////        robot.autonWait(250);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_START_POS);
////        Functions.setShooterMotor(robot, true, Functions.SHOOTER_SPEED - 150);
////        robot.autonWait(1000);
//
////        roadrunner.turn(ANGLE);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_END_POS);
////        robot.autonWait(500);
////        for(Servo s : robot.getServos(ComponentArea.SHOOTER))
////            s.get().setPosition(SHOOTER_SERVO_START_POS);
////
////        robot.autonWait(250);
////
//
//
        double x = safeDistanceTrajectory2.end().getX(), y = safeDistanceTrajectory2.end().getY();

        Trajectory moveToLine = roadrunner.trajectoryBuilder(safeDistanceTrajectory2.end())
                .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, y - 6), Math.toRadians(0))
                .build();

        roadrunner.followTrajectory(moveToLine);


        robot.stopAllThreads();
        PoseStorage.currentPose = roadrunner.getPoseEstimate();
//
    }


    public enum Team {
        RED, BLUE
    }
}


