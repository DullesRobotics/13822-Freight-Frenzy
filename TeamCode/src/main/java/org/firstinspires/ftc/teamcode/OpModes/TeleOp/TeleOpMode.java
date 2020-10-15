package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotManager.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        Robot robot = new Drivetrain(this);
        waitForStart();

        //init parts

        while (opModeIsActive()) {}
        robot.stopAllThreads();
    }
}
