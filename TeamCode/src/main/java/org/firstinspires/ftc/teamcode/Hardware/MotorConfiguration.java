package org.firstinspires.ftc.teamcode.Hardware;

public class MotorConfiguration {

    private final MotorType mt;
    private final double wheelDiameter;
    private final boolean canStrafe;
    private double driveGearReduction = 1;

    /**
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param canStrafe If the wheel can strafe
     */
    public MotorConfiguration(MotorType mt, double wheelDiameter, boolean canStrafe){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.canStrafe = canStrafe;
    }

    /**
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param driveGearReduction Gearbox multiplier (eg 40:1 is 40, 20:1 is 20)
     * @param canStrafe If the wheel can strafe
     */
    public MotorConfiguration(MotorType mt,  double wheelDiameter, double driveGearReduction, boolean canStrafe){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.driveGearReduction = driveGearReduction;
        this.canStrafe = canStrafe;
    }

    /**
     * The type of motor used on the wheel
     * Basically a list of motor types with pre-defined counts
     */
    public enum MotorType {

        CORE_HEX_MOTOR(288),
        HD_HEX_MOTOR(28),
        NEVEREST_ORBITAL(537.6);

        private final double countsPerRevolution;

        MotorType(double countsPerRevolution) {
            this.countsPerRevolution = countsPerRevolution;
        }

        public double countsPerRev() {
            return countsPerRevolution;
        }
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
}
