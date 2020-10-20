package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Acts as a shell around DCMotor, providing more information and automating more processes surrounding it.
 */
public class Motor extends HardwareComponent {

    private boolean isEncoded = false, isOpposite = false;
    private MotorConfiguration motorConfiguration;

    /**
     * @param op The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     * @param motorConfiguration The motor configuration, including the counts per inch and more
     * @param isEncoded Whether or not the motor is encoded
     * @param isOpposite If the motor should be inverted
     */
    public Motor(LinearOpMode op, String id, HardwareComponentArea componentArea, MotorConfiguration motorConfiguration, boolean isEncoded, boolean isOpposite)
    {
        super(op, id, componentArea);
        this.isEncoded = isEncoded;
        this.isOpposite = isOpposite;
        this.motorConfiguration = motorConfiguration;
        get().setDirection(isOpposite ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
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
     * Without a provided motorConfiguration, the motor cannot be encoded
     * @param op The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     * @param isOpposite If the motor should be inverted
     */
    public Motor(LinearOpMode op, String id, HardwareComponentArea componentArea, boolean isOpposite)
    {
        super(op, id, componentArea);
        this.op = op;
        this.isOpposite = isOpposite;
        get().setDirection(isOpposite ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
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
     * @param isOpposite Whether or not the robot is inverted
     */
    public void setOpposite(boolean isOpposite){
        this.isOpposite = isOpposite;
        get().setDirection(isOpposite ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /**
     * @return Whether or not the robot is inverted
     */
    public boolean isOpposite(){
        return isOpposite;
    }

    public MotorConfiguration getMotorConfiguration(){ return motorConfiguration; }

    /**
     * clones the same motor with a different ID
     * @param newID the new ID of the component
     * @return The new motor with the same traits
     */
    public Motor clone(String newID){
        return new Motor(op, newID, getComponentArea(), motorConfiguration, isEncoded, isOpposite);
    }

    /**
     * stops and resets the motor encoder, and then resets to the original motor mode
     */
    public void stopAndResetEncoder(){
        DcMotor.RunMode _tempM = get().getMode();
        get().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        get().setMode(_tempM);
    }

}
