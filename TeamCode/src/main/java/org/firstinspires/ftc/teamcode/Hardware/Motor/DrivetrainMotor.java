package org.firstinspires.ftc.teamcode.Hardware.Motor;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

public class DrivetrainMotor extends Motor {

    private final MotorConfiguration motorConfiguration;
    private final MotorType.DrivetrainPosition drivetrainPosition;

    public DrivetrainMotor(Robot r, String id, MotorConfiguration motorConfiguration, boolean isEncoded, MotorType.DrivetrainPosition drivetrainPosition) {
        super(r, id, ComponentArea.DRIVE_TRAIN, isEncoded);
        this.motorConfiguration = motorConfiguration;
        this.drivetrainPosition = drivetrainPosition;

        switch (drivetrainPosition) {
            case FRM:
            case BRM:
                setFlipped(true);
        }

    }

    /**
     * Get the motor configuration of the drivetrain motor
     * @return The drivetrain motor configuration
     */
    public MotorConfiguration getConfiguration() {
        return motorConfiguration;
    }

    /**
     * Get the drivetrain position (FLM, FRM, BLM, or BRM)
     * @return The position of the motor on the drivetrain
     */
    public MotorType.DrivetrainPosition getDrivetrainPosition() {
        return drivetrainPosition;
    }

    /**
     * @return If the motor is flipped for strafing
     */
    @Deprecated
    public boolean isStrafeFlipped() {
        return drivetrainPosition == MotorType.DrivetrainPosition.FRM || drivetrainPosition == MotorType.DrivetrainPosition.BLM;
    }

    /**
     * clones the same motor with a different ID
     * @param newID the new ID of the component
     * @param drivetrainPosition The new drivetrain position of the component
     * @return The new motor with the same traits
     */
    public DrivetrainMotor clone(String newID, MotorType.DrivetrainPosition drivetrainPosition){
        return new DrivetrainMotor(r, newID, motorConfiguration, isEncoded(), drivetrainPosition);
    }

}
