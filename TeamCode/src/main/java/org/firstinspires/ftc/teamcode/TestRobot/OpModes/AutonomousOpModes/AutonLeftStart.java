package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;

@Autonomous
public class AutonLeftStart extends LinearOpMode {

    StandardDriveTrain robot;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        //sets default drivetrain motor configuration for each motor
//        MotorConfiguration mC = new MotorConfiguration(MotorType.CORE_HEX_MOTOR,true, true, 2.9528, 1);
        //creates central drivetrain object
        robot = new StandardDriveTrain(this, new PID(0,0,0));
        robot.addHardware(Configurator.getHardware(robot));
        //sets motor configuration
//        robot.setAutonMotorConfiguration(mC);
//        //sets max speeds/accelerations in inches/sec
//        robot.setVelocityConstants(30, 30, 60, 60);
//        //sets axis that the control hub is facing
//        robot.setIMUAxis(Axis.Z);
//        //sets how far apart parallel motors are
//        robot.setTrackWidth(11);
        //creates road runner with the information above
        //drive = new SampleMecanumDrive(hardwareMap);

        //UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        //robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam()));

        //Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
        //        .forward(60)
        //        .build();

        waitForStart();

        if (isStopRequested()) return;

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

        //drive.followTrajectory(trajectory);

        //Pose2d poseEstimate = drive.getPoseEstimate();
        //telemetry.addData("finalX", poseEstimate.getX());
        //telemetry.addData("finalY", poseEstimate.getY());
        //telemetry.addData("finalHeading", poseEstimate.getHeading());
        //telemetry.update();

        /*
            Each tile is 2 feet by 2 feet, so it must drive up 3-5 tiles.
            From the leftmost starting position:
                For Tile A (0 rings): 72 inches forward, 36 inches right
                For Tile B (1 ring): 96 inches forward, 12 inches right
                For Tile C (4 rings): 120 inches forward, 36 inches right
            From the rightmost starting position:
                For Tile A: 72 inches forward, 12 inches right
                For Tile B: 96 inches forward, 12 inches left
                For Tile C: 120 inches forward, 12 inches right
         */

//
//        if(pipeline.getAmount() == UltimateGoalPipeline.RingAmount.NONE){
//            robot.autoStraightEncodedPID(72, 3);
//            robot.autoStrafeEncodedPID(12, 3);
//            robot.autoStraightEncodedPID(6, 3);
//            /* Subtracts 8 to correct for it moving 12 left earlier. */
//            robot.autoStrafeEncodedPID(-36 - 8, 3);
//        } else if (pipeline.getAmount() == UltimateGoalPipeline.RingAmount.ONE){
//            robot.autoStraightEncodedPID(96, 3);
//            robot.autoStrafeEncodedPID(12, 3);
//            robot.autoStraightEncodedPID(6, 3);
//            /* Subtracts 8 to correct for it moving 12 left earlier. */
//            robot.autoStrafeEncodedPID(-12 - 8, 3);
//        } else {
//            robot.autoStraightEncodedPID(120, 3);
//            robot.autoStrafeEncodedPID(12, 3);
//            robot.autoStraightEncodedPID(6, 3);
//            /* Subtracts 8 to correct for it moving 12 left earlier. */
//            robot.autoStrafeEncodedPID(-36 - 8, 3);
//        }

        robot.autoDriveTimed(500, DriveTrain.Direction.LEFT);

        robot.autonWait(1000);

        while (!isStopRequested() && opModeIsActive());

        robot.stopAllThreads();
    }
}
