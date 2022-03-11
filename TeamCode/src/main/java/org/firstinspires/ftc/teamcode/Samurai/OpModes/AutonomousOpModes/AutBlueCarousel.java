package org.firstinspires.ftc.teamcode.Samurai.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Samurai.AutonFunctions;

@Autonomous
@Config
public class AutBlueCarousel extends LinearOpMode {
    @Override
    public void runOpMode()throws InterruptedException{
        AutonFunctions.startNew(this, AutonFunctions.TeamColor.BLUE, AutonFunctions.FieldPosition.NEAR_CAROUSEL);
    }
}
