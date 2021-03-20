package org.firstinspires.ftc.teamcode.TestRobot.OpModes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

@Config
public class Functions {

    public static float INTAKE_SPEED = 1, SHOOTER_SPEED = 1;
    public static long SHOOTER_INIT_MILLIS = 2000, SHOOTER_WAIT_MILLIS = 8000, SHOOTER_COOLDOWN = 2000;

    /**
     * Handles intake functions
     * Button Y on controller 2 is the toggle for the intake
     * Uses the togglePressed boolean and the current state of the controller to update the "on"
     * variable. It then uses that to update the intake motors' power.
     * @param r The robot the motors are on
     */
    public static void startIntake(Robot r, Controller ctrl){
        r.getLogger().log(Level.INFO, "Starting intake function");
        r.addThread(new Thread(() -> {
            boolean on = false, togglePressed = false;
            while(r.op().opModeIsActive()){
                /* Toggles on variable */
                if(!togglePressed && ctrl.buttonY() || togglePressed && !ctrl.buttonY()) {
                    togglePressed = ctrl.buttonY();
                    on = !on;
                    setIntake(r, on);
                }
            }
        }), true);
    }

    /**
     * Toggles the intake motor
     * @param r The robot to toggle the intake motors with
     * @param on If the intake should be on or off
     */
    public static void setIntake(Robot r, boolean on) {
        for(Motor m : r.getMotors(ComponentArea.INTAKE)) {
            m.get().setPower(on ? INTAKE_SPEED : 0);
            if(m.isEncoded())
                r.getLogger().putData("Motor " + m.getId() + " Velocity", m.getEncoded().getVelocity());
        }
        r.getLogger().putData("Intake Power", on ? INTAKE_SPEED : 0);
    }

    /**
     * Handles shooter functions
     * Button B on controller 2 is the toggle for the shooter
     * Uses the togglePressed boolean and the current state of the controller to update the "on"
     * variable. It then uses that to update the shooter motors' power.
     * @param r The robot the motors are on
     */
    public static void startShooter(Robot r, Controller ctrl){
        r.getLogger().log(Level.INFO, "Starting shooter function");
        r.addThread(new Thread(() -> {
            boolean on = false, init = false, firstShot = false, state = false;
            long initTime = -1, endTime = 0, cooldownTime = 0;
            boolean alreadyPressed = false;
            while(r.op().opModeIsActive()){
                /* When the button is pressed and it's not already on, begin initializing */
                if(ctrl.buttonB() && !on){
                    on = true;
                    init = true;
                    firstShot = true;
                    initTime = System.currentTimeMillis() + SHOOTER_INIT_MILLIS;
                }

                /* If it's on, run the shooter motor(s). Otherwise, set power to 0 */
                for(Motor m : r.getMotors(ComponentArea.SHOOTER))
                    m.get().setPower(on ? SHOOTER_SPEED : 0);

                /* If it's on, still initializing, but init time has passed, stop initializing. */
                if(on && init && System.currentTimeMillis() > initTime) {
                    init = false;
                    endTime = System.currentTimeMillis() + SHOOTER_WAIT_MILLIS;
                    cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                }

                if(on && !init)
                    if(System.currentTimeMillis() > endTime)
                        on = false;
                    else {
                        if(!ctrl.buttonB())
                            alreadyPressed = false;
                        if (System.currentTimeMillis() > cooldownTime)
                            if (firstShot || (ctrl.buttonB() && !alreadyPressed)) {
                                firstShot = false;
                                alreadyPressed = true;
                                cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                                for(Servo s : r.getServos(ComponentArea.SHOOTER)){
                                    s.get().scaleRange(0,1);
                                    state = !state;
                                    s.get().setPosition(state ? 0.5 : 0);
                                }
                            }
                    }


            }
        }), true);
    }

}
