package org.firstinspires.ftc.teamcode.TestRobot.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.TestRobot.HardwareConfigurator;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class AutonLeftStart extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this, HardwareConfigurator.getHardware(robot));

        robot.getLogger().setDynamicDataHeader("Robot Variables");

        UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam()));

        waitForStart();

        /* Give time for robot to calculate just in case */
        robot.autonWait(1000);

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

        if(pipeline.getAmount() == UltimateGoalPipeline.RingAmount.NONE){
            robot.autoStraightEncodedPID(72, new PID(1,1,1), 3);
            robot.autoStrafeEncodedPID(12, new PID(1,1,1), 3);
            robot.autoStraightEncodedPID(6, new PID(1,1,1), 3);
            /* Subtracts 8 to correct for it moving 12 left earlier. */
            robot.autoStrafeEncodedPID(-36 - 8, new PID(1,1,1), 3);
        } else if (pipeline.getAmount() == UltimateGoalPipeline.RingAmount.ONE){
            robot.autoStraightEncodedPID(96, new PID(1,1,1), 3);
            robot.autoStrafeEncodedPID(12, new PID(1,1,1), 3);
            robot.autoStraightEncodedPID(6, new PID(1,1,1), 3);
            /* Subtracts 8 to correct for it moving 12 left earlier. */
            robot.autoStrafeEncodedPID(-12 - 8, new PID(1,1,1), 3);
        } else {
            robot.autoStraightEncodedPID(120, new PID(1,1,1), 3);
            robot.autoStrafeEncodedPID(12, new PID(1,1,1), 3);
            robot.autoStraightEncodedPID(6, new PID(1,1,1), 3);
            /* Subtracts 8 to correct for it moving 12 left earlier. */
            robot.autoStrafeEncodedPID(-36 - 8, new PID(1,1,1), 3);
        }

        robot.autonWait(1000);

        robot.stopAllThreads();
        robot.op().requestOpModeStop();
    }
}
