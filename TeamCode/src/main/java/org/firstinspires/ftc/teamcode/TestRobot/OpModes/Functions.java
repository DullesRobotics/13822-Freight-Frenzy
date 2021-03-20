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

    //lowered for battery life
    public static float INTAKE_SPEED = 0.5f, SHOOTER_SPEED = 0.5f;
    public static long SHOOTER_INIT_MILLIS = 1000, SHOOTER_WAIT_MILLIS = 8000, SHOOTER_COOLDOWN = 2000;

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

                if(togglePressed && !ctrl.buttonY())
                    togglePressed = false;

                /* Toggles on variable */
                r.getLogger().putData("Intake Power", on ? INTAKE_SPEED : 0);
                if(!togglePressed && ctrl.buttonY()) {
                    togglePressed = true;
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
        r.getLogger().log(Level.INFO, "Turning intake motor(s) " + (on ? "on" : "off"));
        for(Motor m : r.getMotors(ComponentArea.INTAKE))
            m.get().setPower(on ? INTAKE_SPEED : 0);
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

        for(Servo s : r.getServos(ComponentArea.SHOOTER)) {
            s.get().scaleRange(0,1);
            s.get().setPosition(0);
        }

        r.addThread(new Thread(() -> {
            boolean on = false, init = false, firstShot = false, state = false;
            long initTime = -1, endTime = 0, cooldownTime = 0;
            boolean alreadyPressed = false;
            while(r.op().opModeIsActive()){
                /* When the button is pressed and it's not already on, begin initializing */

                r.getLogger().putData("isOn", on);
                r.getLogger().putData("init", init);
                r.getLogger().putData("firstShot", firstShot);
                r.getLogger().putData("state", state);
                r.getLogger().putData("initTime", initTime);
                r.getLogger().putData("endTime", endTime);
                r.getLogger().putData("cooldownTime", cooldownTime);
                r.getLogger().putData("alreadyPressed", alreadyPressed);

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
                                //reset timers
                                endTime = System.currentTimeMillis() + SHOOTER_WAIT_MILLIS;
                                cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                                state = !state;
                                for(Servo s : r.getServos(ComponentArea.SHOOTER))
                                    s.get().setPosition(state ? 1 : 0);
                            }
                    }


            }
        }), true);
    }

    /**
     * Starts or stops the shooter motor(s) for auton
     * @param r The robot with the shooter motor
     * @param on If the motor should be turned on or off
     */
    public static void setShooterMotor(Robot r, boolean on) {
        r.getLogger().log(Level.INFO, "Turning Shooter Motor " + (on ? "on" : "off"));
        for(Motor m : r.getMotors(ComponentArea.SHOOTER))
            m.get().setPower(on ? SHOOTER_SPEED : 0);
    }

    /**
     * Calibrates the shooter servo by setting their scale range to (0, 1) and setting their position to 0.
     * @param r The robot with the shooter servo
     */
    public static void calibrateShooterServos(Robot r){
        r.getLogger().log(Level.INFO, "Calibrating Shooter Servo");
        for(Servo s : r.getServos(ComponentArea.SHOOTER)) {
            s.get().scaleRange(0,1);
            s.get().setPosition(0);
        }
    }

    /**
     * Moves shooter servos between 0 and 1
     * @param r The robot with the shooter servo
     */
    public static void moveShooterServos(Robot r){
        r.getLogger().log(Level.INFO, "Moving shooter servos");
        for(Servo s : r.getServos(ComponentArea.SHOOTER)) {
            double currentPosition = Math.round(s.get().getPosition());
            if(currentPosition == 0)
                s.get().setPosition(1);
            else if(currentPosition == 1)
                s.get().setPosition(0);
        }
    }

}
