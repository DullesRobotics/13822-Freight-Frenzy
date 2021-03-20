package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.Nullable;

/**
 * Acts as a shell around a hardware component to more easily store and reference information about it
 */
public abstract class HardwareComponent {

    /** Op mode */
    protected Robot r;
    /** The hardware this stores */
    protected Object component;
    /** The ID of the motor in the hardwareMap */
    private final String id;
    /** What area this component controls */
    private final ComponentArea componentArea;

    /**
     * Constructs this component without a predefined hardware map entry
     * @param id The id of this component
     * @param componentArea The area of the robot this component controls
     */
    public HardwareComponent(Robot r, String id, ComponentArea componentArea){
        this.r = r;
        this.id = id;
        this.componentArea = componentArea;
    }

    /**
     * Constructs this component with a predefined hardware map entry
     * @param id The id of this component
     * @param component The raw hardware reference
     * @param componentArea The area of the robot this component controls
     */
    public HardwareComponent(Robot r, String id, Object component, ComponentArea componentArea){
        this.r = r;
        this.id = id;
        this.component = component;
        this.componentArea = componentArea;
    }

    /**
     * Gets the raw component type of the hardware. <br\>
     * e.g. DCMotor for Motor
     * @return The raw component type
     */
    @Nullable
    public abstract Object get();

    /**
     * @return The id of this component in on the hardware map
     */
    public String getId(){
        return id;
    }

    /**
     * Where this motor is on the robot
     * @return Where this motor is on the robot
     */
    public ComponentArea getComponentArea(){
        return componentArea;
    }

    /**
     * Set the hardware this component stores
     * @param component The hardware this stores
     */
    protected void setComponent(Object component){
        this.component = component;
    }

    /** @return The textual representation of this hardware component */
    public String toString(){
        return "ID: " + getId() +  ", Component Area: " +  getComponentArea();
    }

}
