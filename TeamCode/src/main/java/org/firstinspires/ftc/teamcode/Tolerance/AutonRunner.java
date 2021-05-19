package org.firstinspires.ftc.teamcode.Tolerance;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.RingDetectionPipeline;

import static org.firstinspires.ftc.teamcode.Tolerance.AutonConstants.*;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.*;
import static org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.RingDetectionPipeline.RingAmount.*;

public class AutonRunner {

    private static volatile MecanumDriveTrain robot;
    private static volatile SampleMecanumDrive roadrunner;
    public static volatile Team team = Team.BLUE;

    /**
     * Starts autonomous
     * @param op The opmode that initiated this
     * @param startPoint The starting position of the robot
     * @param team The robot's team
     * @param side The robot's side
     */
    public static void start(LinearOpMode op, Point startPoint, Team team, Side side)
    {
        AutonRunner.team = team;
        roadrunner = new SampleMecanumDrive(op);
        robot = roadrunner.getDriveTrain();

        robot.addThread(new Thread(() -> {
            while (op.opModeIsActive() && !op.isStopRequested())
                robot.getLogger().updateLog();
        }), true);

        /* Calculating initial position and rotation */
//        double startingRadToTurn = Math.toRadians((side == Side.LEFT ? 1 : -1) * START_STACK_ANGLE);
        Pose2d startPose = new Pose2d(startPoint.y, -startPoint.x, 0);

        roadrunner.setPoseEstimate(startPose);

        /* start OpenCV */
        RingDetectionPipeline pipeline = new RingDetectionPipeline();
        EasyOpenCV ez = new EasyOpenCV(pipeline, robot.getUSBWebcam(), OPEN_CV_CAM_ROTATION);
        robot.addOnManager().initAndStartAddOn(ez);
        setWebcamServo(robot, true, ez, pipeline, null);

        Point zonePoint;

        /* Wait for beginning of auton */
        op.waitForStart();

        if (op.isStopRequested()) return;

        /* ensures claw gets a grip */
        Functions.setClawServos(robot, false);

        /* rotate towards start stack */
//        roadrunner.turn(startingRadToTurn);

        /*
         * NONE - Zone A (lowest)
         * ONE - Zone B (middle)
         * FOUR - Zone C (top)
         */
        RingDetectionPipeline.RingAmount amount = pipeline.getAmount();

        switch (amount) {
            case ONE: zonePoint = ZONE_B; break;
            case FOUR: zonePoint = ZONE_C; break;
            case NONE:
            default: zonePoint = ZONE_A; break;
        }

        int zoneAngle = amount == NONE || amount == FOUR ? -90 : 90;
        zoneAngle = (team == Team.RED) ? -zoneAngle : zoneAngle; //inverse if red

        Trajectory wobbleTrajectory = roadrunner.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(zonePoint.y, (team == Team.RED ? 1 : -1) * zonePoint.x, Math.toRadians(zoneAngle)), 0)
                .build();

        roadrunner.followTrajectory(wobbleTrajectory);

        moveClaw(robot, true);

        robot.autonWait(1000);

        Functions.setClawServos(robot, true);

        robot.autonWait(750);

        moveClaw(robot, false);

        robot.autonWait(600);

        Motor m = robot.getMotor("CLM");
        m.get().setPower(0);

        robot.autonWait(100);

        Point shootingPoint = (team == Team.RED ? SHOOTING_POSITION_RED : SHOOTING_POSITION_BLUE);

        Trajectory shootingTrajectory = roadrunner.trajectoryBuilder(wobbleTrajectory.end())
                .splineToLinearHeading(new Pose2d(shootingPoint.y, -shootingPoint.x, Math.toRadians(SHOOTING_ANGLE_BLUE)), 0)
                .build();

        roadrunner.followTrajectory(shootingTrajectory);

        Functions.calibrateShooterServos(robot);

        int reversing = team == Team.BLUE ? 1 : -1;

        roadrunner.turn(Math.toRadians(reversing * 5));

        shootOnce(robot);

        roadrunner.turn(Math.toRadians(reversing * -5));

        shootOnce(robot);

        roadrunner.turn(Math.toRadians(reversing * -10));

        shootOnce(robot);

        Trajectory moveToLine = roadrunner.trajectoryBuilder(shootingTrajectory.end().plus(new Pose2d(0,0, Math.toRadians(-10))))
            .splineToConstantHeading(new Vector2d(LAUNCH_LINE_Y_COORDINATE, shootingTrajectory.end().getY() - 4), 0)
            .build();

        roadrunner.followTrajectory(moveToLine);

        robot.stopAllThreads();

    }

    private static void moveClaw(Robot r, boolean down) {
        Motor m = r.getMotor("CLM");
        m.get().setPower(0);
        m.get().setDirection(down ? DcMotorSimple.Direction.FORWARD : DcMotor.Direction.REVERSE);
        m.get().setTargetPosition(down ? CLAW_MOVE_DOWN_TICKS : CLAW_MOVE_UP_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);
    }

    private static void shootOnce(Robot r){
        Functions.setShooterMotor(r, true);
        robot.autonWait(1750);
        for(Servo s : r.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        robot.autonWait(250);
        for(Servo s : r.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
    }

    public enum Side {
        LEFT, RIGHT
    }

    public enum Team {
        RED, BLUE
    }
}

