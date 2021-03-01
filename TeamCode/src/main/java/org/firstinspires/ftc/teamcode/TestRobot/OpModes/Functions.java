package org.firstinspires.ftc.teamcode.TestRobot.OpModes;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

public class Functions {

    private static final float INTAKE_SPEED = 1, SHOOTER_SPEED = 2;
    private static final long SHOOTER_INIT_MILLIS = 2000, SHOOTER_WAIT_MILLIS = 8000, SHOOTER_COOLDOWN = 2000;

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
            boolean on = false, init = false, firstShot = false;
            long initTime = -1, endTime = 0, cooldownTime = 0;
            boolean alreadyPressed = false;
            while(r.op().opModeIsActive()){
                /* When the button is pressed and it's not already on, begin initializing */
                if(r.ctrl2().buttonB() && !on){
                    on = true;
                    init = true;
                    firstShot = true;
                    initTime = System.currentTimeMillis() + SHOOTER_INIT_MILLIS;
                }

                /* If it's on, run the shooter motor(s). Otherwise, set power to 0 */
                for(Motor m : r.getMotors(HardwareComponentArea.SHOOTER))
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
                        if(!r.ctrl2().buttonB())
                            alreadyPressed = false;
                        if (System.currentTimeMillis() > cooldownTime)
                            if (firstShot || (r.ctrl2().buttonB() && !alreadyPressed)) {
                                firstShot = false;
                                alreadyPressed = true;
                                cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                                for(Servo s : r.getServos(HardwareComponentArea.SHOOTER)){
                                    s.get().scaleRange(0,1);

                                    /* must keep the position within 0 & 1 */
                                    double newPosition = s.get().getPosition() + 0.5;
                                    if(newPosition >= 1)
                                        newPosition -= 1;

                                    s.get().setPosition(newPosition);
                                }
                            }
                    }


            }
        }), true);
    }

}
