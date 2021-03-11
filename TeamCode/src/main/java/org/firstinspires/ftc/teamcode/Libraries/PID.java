package org.firstinspires.ftc.teamcode.Libraries;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {

    private volatile double kp = 0, ki = 0, kd = 0;
    private double errorSum = 0, prevError = 0, prevTime = 0;

    /**
     * tinker with these values, keep them small
     * @param kp How much we should adjust movement to ensure a smooth slowdown
     * @param ki Keeps track of how close we are, keeps getting bigger to counteract kp slowing down quickly
     * @param kd Keeps rest of calculations in check to ensure it doesn't overshoot
     */
    public PID(double kp, double ki, double kd){
        updateConstants(kp, ki, kd);
    }

    public void updateConstants(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Calculates the current power the motor should move to meet the specified target angle
     * to ensure the robot does not overshoot
     * @param currentAngle The current angle of the robot
     * @param targetAngle The target angle of the robot
     * @return The power the motors should have to move to the target angle currently
     */
    public double update(PIDType pidType, double currentAngle, double targetAngle){
        if(prevTime == 0) prevTime = System.currentTimeMillis();
        double error = getError(pidType, currentAngle, targetAngle);
        double dt = System.currentTimeMillis() - prevTime;
        errorSum += (error * dt);

        /* PID Calculations */
        double newPower = 0;
        /* How much we should adjust movement to ensure a smooth slowdown */
        newPower += error * kp;
        /* Keeps track of how close we are, keeps getting bigger to counteract kp slowing down quickly */
        newPower += errorSum * ki;
        /* Keeps rest of calculations in check to ensure it doesn't overshoot */
        newPower += ((prevError - error) / dt) * kd;
        /* //////////////// */

        prevError = error;
        prevTime = System.currentTimeMillis();
        return newPower;
    }

    /**
     * Gets the error, aka how much we need to move to get to the target angle
     * @param currentAngle The current angle of the robot
     * @param targetAngle The target angle of the robot
     * @return The difference between the two angles, in a -180º to 180º form
     */
    private static double getError(PIDType pidType, double currentAngle, double targetAngle){
        double diff = currentAngle - targetAngle;
        switch(pidType) {
            case ONE_EIGHTY_ANGLE:
                diff = AngleUnit.normalizeDegrees(diff);
                break;
            case THREE_SIXTY_ANGLE:
                diff = diff < 0 ? diff + 360 : diff;
                break;
        }
        return diff;
    }

    /** Supported variable types of PID */
    public enum PIDType {
        ONE_EIGHTY_ANGLE, //preferred
        THREE_SIXTY_ANGLE,
        NUMBER;
    }

    /**
     * How much we should adjust movement to ensure a smooth slowdown
     * @return the P variable in PID
     */
    public double getKp() {
        return kp;
    }

    /**
     * Keeps track of how close we are, keeps getting bigger to counteract kp slowing down quickly
     * @return The I variable in PID
     */
    public double getKi() {
        return ki;
    }

    /**
     * Keeps rest of calculations in check to ensure it doesn't overshoot
     * @return The D variable in PID
     */
    public double getKd() {
        return kd;
    }
}
