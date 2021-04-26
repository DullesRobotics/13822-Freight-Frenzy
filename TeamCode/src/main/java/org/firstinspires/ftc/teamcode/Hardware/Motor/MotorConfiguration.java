package org.firstinspires.ftc.teamcode.Hardware.Motor;

public class MotorConfiguration {

    private final MotorType mt;
    private final double wheelDiameter, gearRatio;
    private final boolean canStrafe;

    /**
     * For non-drivetrain motors
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     */
    public MotorConfiguration(MotorType mt, double wheelDiameter){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.canStrafe = false;
        this.gearRatio = 1;
    }

    /**
     * For drivetrain motors
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param canStrafe If the wheel can strafe
     * @param gearRatio GEAR_RATIO is the ratio of the output (wheel) speed to input (motor) speed.
     *                  If you are using direct drive—no gears/belts—GEAR_RATIO should be 1.
     *                  A gear ratio more than 1 will indicate that your wheel spins faster
     *                  than your motor. A gear ratio less than one will indicate that your wheel
     *                  spins slower than your motor. For example, the 2019 v1 goBILDA strafer kit
     *                  includes a set of 1:2 bevel gears, reducing your output speed by half.
     *                  So your gear ratio will be 1/2 or 0.5
     */
    public MotorConfiguration(MotorType mt, boolean canStrafe, double wheelDiameter, double gearRatio){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.canStrafe = canStrafe;
        this.gearRatio = gearRatio;
    }

    /**
     * @return The counts per inch of the motor configuration. Only useful in encoded motors
     */
    public double countsPerInch(){
        return (mt.countsPerRev() * getMotorType().getDriveGearReduction()) / (getWheelDiameter() * Math.PI);
    }

    /**
     * @return The amount of counts for a specified amount of inches
     */
    public int inchesToCounts(double inches){
        return ((int)(inches * countsPerInch()));
    }

    /**
     * @return The type of motor being used
     */
    public MotorType getMotorType() {
        return mt;
    }

    /**
     * @return If the wheel can strafe
     */
    public boolean canStrafe(){ return canStrafe; }

    /**
     * @return The diameter of the wheel in inches
     */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * GEAR_RATIO is the ratio of the output (wheel) speed to input (motor) speed.
     * If you are using direct drive—no gears/belts—GEAR_RATIO should be 1.
     * A gear ratio more than 1 will indicate that your wheel spins faster
     * than your motor. A gear ratio less than one will indicate that your wheel
     * spins slower than your motor. For example, the 2019 v1 goBILDA strafer kit
     * includes a set of 1:2 bevel gears, reducing your output speed by half.
     * So your gear ratio will be 1/2 or 0.5
     * @return The gear ratio of the motor, as described above.
     */
    public double getGearRatio() {
        return gearRatio;
    }

}
