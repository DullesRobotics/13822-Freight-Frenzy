package org.firstinspires.ftc.teamcode.TestRobot.OpModes.TeleOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.StandardDriveTrain;
import org.firstinspires.ftc.teamcode.TestRobot.Configurator;
import org.firstinspires.ftc.teamcode.TestRobot.OpModes.Functions;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {

    private MecanumDriveTrain robot;
    public static OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveTrain(this, new PID(0,0,0));
        robot.addHardware(new Motor(robot, "FLM", ComponentArea.INTAKE, true));

        waitForStart();

        /* Robot functions */
        Functions.startIntake(robot, robot.ctrl1());

        while (opModeIsActive())
            robot.getLogger().updateLog();

        robot.stopAllThreads();
    }
}
