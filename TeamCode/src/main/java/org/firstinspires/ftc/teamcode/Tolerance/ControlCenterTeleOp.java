package org.firstinspires.ftc.teamcode.Tolerance;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.UUID;
import java.util.logging.Level;

public class ControlCenterTeleOp {

    static final double motorCarouselSpeed = 0.5;

    public static void carouselSpin(Robot r, Controller ctrl, boolean isRed){
        UUID uuid = r.addThread(new Thread(() -> {
            boolean on = false, alreadyPressed = false;
            r.getLogger().log(Level.INFO, "Turning carousel motor " + (on ? "on" : "off"));
            while (r.op().opModeIsActive()) {

                if (alreadyPressed && !ctrl.leftBumper())
                    alreadyPressed = false;

                if ((ctrl.leftBumper()) && !on && !alreadyPressed) {
                    alreadyPressed = true;
                    on = true;
                    for(Motor m : r.getMotors(ComponentArea.CAROUSEL)) {
                        m.get().setPower(on ? isRed ? motorCarouselSpeed : -motorCarouselSpeed : 0);
                    }
                }
                if((ctrl.leftBumper() && on && !alreadyPressed)){
                    alreadyPressed = true;
                    on = false;

                }
            }
        }), true);
    }
}
