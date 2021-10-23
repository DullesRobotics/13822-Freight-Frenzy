package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
public class AutonBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode()throws InterruptedException{
        AutonFunctions.start(this, AutonFunctions.TeamColor.blue, AutonFunctions.Direction.left);

    }
}
