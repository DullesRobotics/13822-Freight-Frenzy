package org.firstinspires.ftc.teamcode.Samurai.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Samurai.AutonFunctions;

@Autonomous
@Config
public class AutonRedLeft extends LinearOpMode {
    @Override
    public void runOpMode()throws InterruptedException{
        AutonFunctions.start(this, AutonFunctions.TeamColor.RED, AutonFunctions.Direction.LEFT);
    }
}
