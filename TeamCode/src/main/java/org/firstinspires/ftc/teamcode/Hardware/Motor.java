package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Acts as a shell around DCMotor, providing more information and automating more processes surrounding it.
 */
public class Motor extends HardwareComponent {

    private boolean isEncoded = false, canStrafe = false;
    private MotorConfiguration motorConfiguration;
    private LinearOpMode op;

    /**
     * @param op The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     * @param motorConfiguration The motor configuration, including the counts per inch and more
     * @param canStrafe Whether or not the motor can strafe
     * @param isEncoded Whether or not the motor is encoded
     */
    public Motor(LinearOpMode op, String id, HardwareComponentArea componentArea, MotorConfiguration motorConfiguration, boolean isEncoded, boolean canStrafe)
    {
        super(id, componentArea);
        this.op = op;
        this.isEncoded = isEncoded;
        this.canStrafe = canStrafe;
        this.motorConfiguration = motorConfiguration;
        try {
            setComponent(op.hardwareMap.dcMotor.get(id));
            op.hardwareMap.dcMotor.get(id).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            op.telemetry.addData("Error Adding Encoded Motor " + id + ":", e);
            op.telemetry.update();
            op.requestOpModeStop();
        }
    }

    /**
     * @param op The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     * @param canStrafe Whether or not the motor can strafe
     */
    public Motor(LinearOpMode op, String id, HardwareComponentArea componentArea, boolean canStrafe)
    {
        super(id, componentArea);
        this.op = op;
        this.canStrafe = canStrafe;
        try {
            setComponent(op.hardwareMap.dcMotor.get(id));
            op.hardwareMap.dcMotor.get(id).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            op.telemetry.addData("Error Adding Non-encoded Motor " + id + ":", e);
            op.telemetry.update();
            op.requestOpModeStop();
        }
    }

    @Override
    public DcMotor get() {
        return (DcMotor) component;
    }

    /**
     * if the motor is encoded
     * @return if the motor is encoded
     */
    public boolean isEncoded(){
        return isEncoded;
    }

    /**
     * if the motor can strafe
     * @return if the motor can strafe
     */
    public boolean canStrafe(){
        return canStrafe;
    }

    /**
     * clones the same motor with a different ID
     * @param newID the new ID of the component
     * @return The new motor with the same traits
     */
    public Motor clone(String newID){
        return new Motor(op, newID, getComponentArea(), motorConfiguration, isEncoded, canStrafe);
    }

}
