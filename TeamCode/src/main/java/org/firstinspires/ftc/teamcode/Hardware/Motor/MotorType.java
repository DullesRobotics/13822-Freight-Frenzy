package org.firstinspires.ftc.teamcode.Hardware.Motor;

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
