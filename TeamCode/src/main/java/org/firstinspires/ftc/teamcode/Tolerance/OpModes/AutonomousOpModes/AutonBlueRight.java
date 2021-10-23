package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class AutonBlueRight {
    @Override
    public void runOpMode()throws InterruptedException{
        AutonFunctions.start(this, AutonFunctions.TeamColor.blue, AutonFunctions.Direction.right);
    }
}
