package org.firstinspires.ftc.teamcode.Hardware;

public class MotorConfiguration {

    private final MotorType mt;
    private final double wheelDiameter;
    private double driveGearReduction = 1;

    /**
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     */
    public MotorConfiguration(MotorType mt, double wheelDiameter){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
    }

    /**
     * @param mt The type of motor hardware being used
     * @param wheelDiameter The diameter of the wheel in INCHES
     * @param driveGearReduction
     */
    public MotorConfiguration(MotorType mt,  double wheelDiameter, double driveGearReduction){
        this.mt = mt;
        this.wheelDiameter = wheelDiameter;
        this.driveGearReduction = driveGearReduction;
    }

    /**
     * The type of motor used on the wheel
     * Basically a list of motor types with pre-defined counts
     */
    public enum MotorType {

        CORE_HEX_MOTOR(288),
        HD_HEX_MOTOR(28),
        NEVEREST_SERIES(28);

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
     * @return The type of motor being used
     */
    public MotorType getMotorType() {
        return mt;
    }

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
