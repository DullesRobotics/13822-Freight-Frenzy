package org.firstinspires.ftc.teamcode.Hardware.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.Nullable;

import java.util.logging.Level;

/**
 * Acts as a shell around DCMotor, providing more information and automating more processes surrounding it.
 */
public class Motor extends HardwareComponent {

    private boolean isFlipped, isEncoded;

    /**
     * Without a provided motorConfiguration, the motor cannot be encoded
     * @param r The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     */
    public Motor(Robot r, String id, ComponentArea componentArea, boolean isEncoded)
    {
        super(r, id, componentArea);
        this.isEncoded = isEncoded;
        try {
            setComponent(r.op().hardwareMap.get(isEncoded ? DcMotorEx.class : DcMotor.class, id));
            if(isEncoded)
                get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            r.getLogger().log(Level.SEVERE, "Error Adding Motor " + id, e.toString());
            r.op().requestOpModeStop();
        }
    }

    @Override
    public DcMotor get() {
        return (DcMotor) component;
    }

    @Nullable
    public DcMotorEx getEncoded() {
        return isEncoded() ? (DcMotorEx) component : null;
    }

    /**
     * @param isFlipped Whether or not the robot is inverted
     */
    public Motor setFlipped(boolean isFlipped){
        this.isFlipped = isFlipped;
        get().setDirection(isFlipped ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return this;
    }

    /**
     * Usually a right wheel
     * @return Whether or not the robot is inverted
     */
    public boolean isFlipped(){
        return isFlipped;
    }

    /**
     * @return If the motor is encoded
     */
    public boolean isEncoded(){
        return isEncoded;
    }

    /**
     * clones the same motor with a different ID
     * @param newID the new ID of the component
     * @return The new motor with the same traits
     */
    public Motor clone(String newID){
        return new Motor(r, newID, getComponentArea(), isEncoded);
    }

    /**
     * stops and resets the motor encoder, and then resets to the original motor mode
     */
    public void stopAndResetEncoder(){
        DcMotor.RunMode _tempM = get().getMode();
        get().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        get().setMode(_tempM);
    }

    /** @return The textual representation of this motor */
    @Override
    public String toString(){
        return "ID: " + getId() +  ", Component Area: " +  getComponentArea() +
                ", isFlipped: " + isFlipped() + ", Port: " + get().getPortNumber();
    }

}
