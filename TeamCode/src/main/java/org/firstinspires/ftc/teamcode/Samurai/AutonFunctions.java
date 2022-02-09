package org.firstinspires.ftc.teamcode.Samurai;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

public class AutonFunctions {
    private static volatile MecanumDriveTrain mainFrame;
    private static volatile SampleMecanumDrive roadRunner;
    private static volatile TeamColor team;
    private volatile Point start = startRedRight;

    public static void start(LinearOpMode op, TeamColor t, Direction position){

        AutonFunctions.team = team;
        roadRunner = new SampleMecanumDrive(op);
        mainFrame = roadRunner.getDriveTrain();
        mainFrame.addThread(new Thread(() -> {
            while(op.opModeIsActive() && !op.isStopRequested()){
                mainFrame.getLogger().updateLog();
            }
        }), true);

        double angleStart = (team == TeamColor.blue)? 180:-180;
        team = t;
        Point start = (team == TeamColor.blue) && (position==Direction.left) ? (team == TeamColor.red) && (position==Direction.left) ?startBlueLeft:startBlueRight:startRedLeft;
        Pose2d startPose = new Pose2d(start.x,start.y,angleStart);
        roadRunner.setPoseEstimate(startPose);

        op.waitForStart();
        if(op.isStopRequested()) return;

        //Move Forward to Carousel
        Trajectory carousel = roadRunner.trajectoryBuilder(startPose)
                .forward(-36)
                .build();
        roadRunner.followTrajectory(carousel);
        //Rotate Carousel
        turnCarousel(mainFrame);
    }

    private static void turnCarousel(Robot r) {
        for(Motor m : r.getMotors(ComponentArea.CAROUSEL)) {
            m.get().setPower(.6);
        }
    }
    public enum TeamColor{
        red,blue;
    }
    public enum Direction{
        left,right;
    }
    public static Point
        startRedLeft = new Point(-36,60),
        startRedRight = new Point(12, -60),
        startBlueLeft = new Point(-36, 60),
        startBlueRight = new Point(12, 60);
}

