package org.firstinspires.ftc.teamcode.TestRobot.OpModes.TeleOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.RobotRecorder;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.Functions;
import org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines.UltimateGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
@Config
public class MainMode extends LinearOpMode {

    private StandardDriveTrain robot;
    public static OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new StandardDriveTrain(this, new PID(0,0,0));
        robot.addHardware(Configurator.getHardware(robot));

        /* Add-on initializing */
        //UltimateGoalPipeline pipeline = new UltimateGoalPipeline();
        //robot.addOnManager().initAndStartAddOn(new EasyOpenCV(pipeline, robot.getUSBWebcam(), rotation));
        robot.addOnManager().initAddOn(new RobotRecorder());

        waitForStart();

        /* Add-on starting */
        robot.addOnManager().startAddOn(AddOnType.ROBOT_RECORDER);

        /* Robot functions */
        robot.driveWithController(robot.ctrl1());
        //Functions.startIntake(robot);
        //Functions.startShooter(robot);

        while (opModeIsActive()) {
          //  robot.getLogger().putData("ring analysis", pipeline.getAnalysis() + " (" + pipeline.getAmount().toString() + ")");
            //robot.getLogger().putData("a button pressed", robot.ctrl1().buttonA());
            robot.getLogger().updateLog();
        }

        robot.stopAllThreads();
    }
}
