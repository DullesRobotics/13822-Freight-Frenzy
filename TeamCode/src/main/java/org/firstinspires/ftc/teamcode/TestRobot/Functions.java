package org.firstinspires.ftc.teamcode.TestRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

@Config
public class Functions {

    //lowered for battery life
    public static double INTAKE_SPEED = 0.7, SHOOTER_SPEED = 1;
    public static int SHOOTER_INIT_MILLIS = 2000, SHOOTER_WAIT_MILLIS = 4000, SHOOTER_COOLDOWN = 1500;
    public static double SHOOTER_SERVO_START_POS = 0.51, SHOOTER_SERVO_END_POS = 0.66;
    public static double CLAW_SERVO_CLOSED_POS = 0, CLAW_SERVO_OPEN_POS = 0.58;
    public static double CLAW_SERVO_CLOSED_POS_2 = 0.06, CLAW_SERVO_OPEN_POS_2 = 0.58;
    public static int CLAW_MOTOR_MID_TICKS = 900, CLAW_MOTOR_END_TICKS = 500;
    public static double CLAW_MOTOR_PWR = 0.7;
    public static int TIME_TO_MOVE = 500;

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
            boolean on = false, togglePressed = false, forward = true;
            while(r.op().opModeIsActive()){

                if(ctrl.buttonUp())
                    forward = true;
                else if(ctrl.buttonDown())
                    forward = false;

                if(togglePressed && !ctrl.buttonY())
                    togglePressed = false;

                /* Toggles on variable */
                r.getLogger().putData("Intake Power", on ? INTAKE_SPEED : 0);
                if(!togglePressed && ctrl.buttonY()) {
                    togglePressed = true;
                    on = !on;
                    //do something`
                    setIntake(r, on, forward);
                }
            }
        }), true);
    }

    /**
     * Toggles the intake motor
     * @param r The robot to toggle the intake motors with
     * @param on If the intake should be on or off
     */
    public static void setIntake(Robot r, boolean on, boolean forward) {
        r.getLogger().log(Level.INFO, "Turning intake motor(s) " + (on ? "on" : "off"));
        for(Motor m : r.getMotors(ComponentArea.INTAKE))
            m.get().setPower(on ? forward ? INTAKE_SPEED : -INTAKE_SPEED : 0);
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

        calibrateShooterServos(r);

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

                if(ctrl.buttonX() && !on){
                    on = true;
                    init = true;
                    firstShot = true;
                    initTime = System.currentTimeMillis() + SHOOTER_INIT_MILLIS;
                    setShooterMotor(r, true);
                }

                /* If it's on, still initializing, but init time has passed, stop initializing. */
                if(on && init && System.currentTimeMillis() > initTime) {
                    init = false;
                    endTime = System.currentTimeMillis() + SHOOTER_WAIT_MILLIS;
                    cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                }

                if(on && !init)
                    if(System.currentTimeMillis() > endTime) {
                        on = false;
                        setShooterMotor(r, false);
                    } else {
                        if(!ctrl.buttonX())
                            alreadyPressed = false;
                        if (System.currentTimeMillis() > cooldownTime)
                            if (firstShot || (ctrl.buttonX() && !alreadyPressed)) {
                                firstShot = false;
                                alreadyPressed = true;
                                //reset timers
                                endTime = System.currentTimeMillis() + SHOOTER_WAIT_MILLIS;
                                cooldownTime = System.currentTimeMillis() + SHOOTER_COOLDOWN;
                                //state = !state;
                                useShooterServos(r);
                            }
                    }
            }
        }), true);
    }

    //58in left of 0,0 blue


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
     * Starts or stops the shooter motor(s) for auton
     * @param r The robot with the shooter motor
     * @param on If the motor should be turned on or off
     */
    public static void setShooterMotor(Robot r, boolean on, double power) {
        r.getLogger().log(Level.INFO, "Turning Shooter Motor " + (on ? "on" : "off") + " (power=" + power + ")");
        for(Motor m : r.getMotors(ComponentArea.SHOOTER))
            m.get().setPower(on ? power : 0);
    }

    /**
     * Calibrates the shooter servo by setting their scale range to (0, 1) and setting their position to 0.
     * @param r The robot with the shooter servo
     */
    public static void calibrateShooterServos(Robot r){
        r.getLogger().log(Level.INFO, "Calibrating Shooter Servo");
        for(Servo s : r.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
    }

    /**
     * Moves shooter servos between 0 and 1
     * @param r The robot with the shooter servo
     */
    public static void useShooterServos(Robot r){
        r.getLogger().log(Level.INFO, "Using shooter servos");
        final long timeToWait = System.currentTimeMillis() + 500;
        for(Servo s : r.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_END_POS);
        while(r.op().opModeIsActive() && System.currentTimeMillis() < timeToWait) {}
        for(Servo s : r.getServos(ComponentArea.SHOOTER))
            s.get().setPosition(SHOOTER_SERVO_START_POS);
    }

    /**
     * Sets claw servo position between two states
     * @param r The robot with the claw servo
     */
    public static void setClawServos(Robot r, boolean open){
        r.getLogger().log(Level.INFO, "Setting claw servos (open = " + open + ")");
        r.getServo("CLS").get().setPosition(open ? CLAW_SERVO_OPEN_POS : CLAW_SERVO_CLOSED_POS);
        r.getServo("CLS2").get().setPosition(open ? CLAW_SERVO_OPEN_POS_2 : CLAW_SERVO_CLOSED_POS_2);
    }

    public static void setClawArmPosition(Robot r, boolean down) {
        r.getLogger().log(Level.INFO, "Moving claw arm (down = " + down + ")");
        Motor m = r.getMotor("CLM");
////        m.get().setPower(down ? -CLAW_MOTOR_PWR : CLAW_MOTOR_PWR);
////        long timeToStop = System.currentTimeMillis() + TIME_TO_MOVE;
////        while(System.currentTimeMillis() < timeToStop) {}
////        m.get().setPower(0);
//
        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
        m.get().setTargetPosition(down ? CLAW_MOTOR_END_TICKS : CLAW_MOTOR_MID_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.getLogger().log(Level.INFO, m.get().getZeroPowerBehavior() + "");
        r.getLogger().log(Level.INFO, m.get().getMode() + "");
        r.getLogger().putData("Claw Target Position", m.get().getTargetPosition());
        m.get().setPower(CLAW_MOTOR_PWR);
        r.getLogger().putData("Claw Motor Velocity", m.getEncoded().getVelocity());
            while(r.op().opModeIsActive() && !r.op().isStopRequested() && m.get().isBusy())
            {
                r.getLogger().putData("Claw Motor Velocity", m.getEncoded().getVelocity());
            }
            m.get().setPower(0);
    }

    /**
     * Starts claw teleop handling of both the claw servo and motor position
     * @param r The robot with the claw
     */
    public static void startClaw(Robot r, Controller ctrl){
        r.getLogger().log(Level.INFO, "Starting claw function");
        int startingPosition = r.getMotor("CLM").get().getCurrentPosition();
        r.addThread(new Thread(() -> {
            boolean clawOpen = false, toggleClawPressed = false;
            boolean armDown = false, toggleArmPressed = false;
            Motor m = r.getMotor("CLM");
            while(r.op().opModeIsActive()){

                if(toggleClawPressed && !ctrl.leftBumper())
                    toggleClawPressed = false;

                /* Toggles on variable for claw */
                if(!toggleClawPressed && ctrl.leftBumper()) {
                    toggleClawPressed = true;
                    clawOpen = !clawOpen;
                    //do something
                    setClawServos(r, clawOpen);
                }

//                if(toggleArmPressed && !(ctrl.leftTrigger() > 0))
//                    toggleArmPressed = false;
//
//                /* Toggles on variable for arm */
//                if(!toggleArmPressed && ctrl.leftTrigger() > 0) {
//                    toggleArmPressed = true;
//                    armDown = !armDown;
//                    //do something
//                    setClawArmPosition(r, armDown, startingPosition);
//                }

                if(ctrl.leftTrigger() > 0){
                    m.get().setPower( CLAW_MOTOR_PWR);
                } else if (ctrl.rightTrigger() > 0){
                    m.get().setPower(-CLAW_MOTOR_PWR);
                } else {
                    m.get().setPower(0);
                }

            }
        }), true);
    }



}
