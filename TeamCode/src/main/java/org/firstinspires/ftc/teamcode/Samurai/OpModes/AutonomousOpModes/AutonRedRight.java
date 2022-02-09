package org.firstinspires.ftc.teamcode.Samurai.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Samurai.AutonFunctions;

@Autonomous
@Config
public class AutonRedRight extends LinearOpMode {
    @Override
    public void runOpMode()throws InterruptedException{
        AutonFunctions.start(this, AutonFunctions.TeamColor.red, AutonFunctions.Direction.right);
    }
}
