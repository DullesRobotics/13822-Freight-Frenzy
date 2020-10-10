package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.OpModes.OpModeHandler;

@TeleOp
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        Robot robot = new Robot(this);
        OpModeHandler.addHardware(robot, this);
        waitForStart();

        //init parts

        while (opModeIsActive()) {}
        robot.stopAllThreads();
    }
}
