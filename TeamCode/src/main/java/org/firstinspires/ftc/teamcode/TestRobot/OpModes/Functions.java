package org.firstinspires.ftc.teamcode.TestRobot.OpModes;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

public class Functions {

    private static final float INTAKE_SPEED = 1;

    /**
     * Handles intake functions
     * Button Y on controller 2 is the toggle for the intake
     * Uses the togglePressed boolean and the current state of the controller to update the "on"
     * variable. It then uses that to update the intake motors' power.
     * @param r The robot the motors are on
     */
    public static void startIntake(Robot r){
        r.getLogger().log(Level.INFO, "Starting intake function");
        r.addThread(new Thread(() -> {
            boolean on = false, togglePressed = false;
            while(r.op().opModeIsActive()){
                /* Toggles on variable */
                if(!togglePressed && r.ctrl2().buttonY() || togglePressed && !r.ctrl2().buttonY()) {
                    togglePressed = r.ctrl2().buttonY();
                    on = !on;

                    /* When toggled it updates the intake motor(s) */
                    for(Motor m : r.getMotors(HardwareComponentArea.INTAKE))
                        if(on)
                            m.get().setPower(INTAKE_SPEED);
                        else
                            m.get().setPower(0);

                }
            }
        }), true);
    }

    /**
     * Handles shooter functions
     * Button B on controller 2 is the toggle for the shooter
     * Uses the togglePressed boolean and the current state of the controller to update the "on"
     * variable. It then uses that to update the shooter motors' power.
     * @param r The robot the motors are on
     */
    public static void startShooter(Robot r){
        r.getLogger().log(Level.INFO, "Starting shooter function");
        r.addThread(new Thread(() -> {
            boolean on = false, togglePressed = false;
            while(r.op().opModeIsActive()){
                /* Toggles on variable */
                if(!togglePressed && r.ctrl1().buttonB() || togglePressed && !r.ctrl2().buttonB()) {
                    togglePressed = r.ctrl2().buttonB();
                    on = !on;

                    /* When toggled it updates the shooter motor(s) */
                    for(Motor m : r.getMotors(HardwareComponentArea.SHOOTER))
                        if(on)
                            m.get().setPower(INTAKE_SPEED);
                        else
                            m.get().setPower(0);

                }
            }
        }), true);
    }

}
