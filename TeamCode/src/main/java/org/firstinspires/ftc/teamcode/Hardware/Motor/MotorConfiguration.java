package org.firstinspires.ftc.teamcode.Hardware.Motor;

public class MotorConfiguration {

    private final MotorType mt;
    private final double wheelDiameter, maxRPM, driveGearReduction, gearRatio;
    private final boolean canStrafe, isEncoded;

    /**
     * For non-drivetrain motors
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param isEncoded If the motor is encoded
     * @param driveGearReduction Gearbox multiplier (eg 40:1 is 40, 20:1 is 20)
     */
    public MotorConfiguration(MotorType mt, boolean isEncoded, double wheelDiameter, double driveGearReduction){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.isEncoded = isEncoded;
        this.canStrafe = false;
        this.gearRatio = 1;
        this.driveGearReduction = driveGearReduction;
        this.maxRPM = 100;
    }

    /**
     * For drivetrain motors
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param driveGearReduction Gearbox multiplier (eg 40:1 is 40, 20:1 is 20)
     * @param canStrafe If the wheel can strafe
     * @param maxRPM The max RPM of the motor
     * @param isEncoded If the motor is encoded
     * @param gearRatio GEAR_RATIO is the ratio of the output (wheel) speed to input (motor) speed.
     *                  If you are using direct drive—no gears/belts—GEAR_RATIO should be 1.
     *                  A gear ratio more than 1 will indicate that your wheel spins faster
     *                  than your motor. A gear ratio less than one will indicate that your wheel
     *                  spins slower than your motor. For example, the 2019 v1 goBILDA strafer kit
     *                  includes a set of 1:2 bevel gears, reducing your output speed by half.
     *                  So your gear ratio will be 1/2 or 0.5
     */
    public MotorConfiguration(MotorType mt, boolean canStrafe, boolean isEncoded, double wheelDiameter, double driveGearReduction, double maxRPM, double gearRatio){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.driveGearReduction = driveGearReduction;
        this.canStrafe = canStrafe;
        this.isEncoded = isEncoded;
        this.maxRPM = maxRPM;
        this.gearRatio = gearRatio;
    }

    /**
     * @return The counts per inch of the motor configuration. Only useful in encoded motors
     */
    public double countsPerInch(){
        return (mt.countsPerRev() * getDriveGearReduction()) / (getWheelDiameter() * Math.PI);
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
     * @return The drive gear reduction of the motor. The default is 1.0
     */
    public double getDriveGearReduction() {
        return driveGearReduction;
    }

    /**
     * @return The diameter of the wheel in inches
     */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * @return If the wheel supports encoding
     */
    public boolean isEncoded() {
        return isEncoded;
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

    /**
     * The max speed of the motor in revs per second
     * @return The max rpm of the motor
     */
    public double getMaxRPM() {
        return maxRPM;
    }
}
