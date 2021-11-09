package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOn;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnHandler;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines.GreenScanningPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class TestAuton extends LinearOpMode {
    MecanumDriveTrain robot;
    @Override
    public void runOpMode() throws InterruptedException {



        robot = new MecanumDriveTrain(this);
        robot.addHardware(new USBWebcam(robot, "Webcam"));

        robot.addOnManager().initAndStartAddOn(new EasyOpenCV(new GreenScanningPipeline(), robot.getUSBWebcam(), OpenCvCameraRotation.UPRIGHT));

        waitForStart();


        while(opModeIsActive()){}

            robot.addOnManager().stopAddOn(AddOnType.EASY_OPEN_CV);


    }
}
