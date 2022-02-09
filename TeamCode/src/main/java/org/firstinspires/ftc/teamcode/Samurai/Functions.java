package org.firstinspires.ftc.teamcode.Samurai;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.UUID;

public class Functions {

    static final double motorCarouselSpeed = 1;

    public static void carouselSpin(Robot r, Controller ctrl){
        UUID uuid = r.addThread(new Thread(() -> {
            Motor carouselMotor = r.getMotor("CAR");
            while (r.op().opModeIsActive()) {
                carouselMotor.get().setPower(ctrl.buttonX() ? -motorCarouselSpeed : 0);
                carouselMotor.get().setPower(ctrl.buttonY() ? motorCarouselSpeed : 0);
            }
        }), true);
    }

    public static void intakeInOut(Robot r, Controller ctrl){
        UUID uuid = r.addThread(new Thread(() -> {
            Motor intakeMotor = r.getMotor("INT");
            boolean goingForward = false;
            boolean on = false;
            boolean currentlyPressed = false;
            while(r.op().opModeIsActive()){

                //r.getLogger().log(Level.INFO, "on: " + on + ", forward: " + goingForward + ", pressed: " + currentlyPressed);

                if(ctrl.buttonUp() && !currentlyPressed){
                    if(goingForward){
                        on = false;
                    } else {
                        on = true;
                        goingForward = true;
                    }
                    currentlyPressed = true;
                } else if(ctrl.buttonDown() && !currentlyPressed){
                    if(!goingForward){
                        on = false;
                    } else {
                        on = true;
                        goingForward = false;
                    }
                    currentlyPressed = true;
                }

                if(currentlyPressed && !ctrl.buttonDown() && !ctrl.buttonUp()){
                    currentlyPressed = false;
                }

                if(on){
                    if(goingForward){
                        intakeMotor.get().setPower(0.75);
                    } else {
                        intakeMotor.get().setPower(-0.75);
                    }
                } else {
                    intakeMotor.get().setPower(0);
                }
            }
        }), true);
    }

}
