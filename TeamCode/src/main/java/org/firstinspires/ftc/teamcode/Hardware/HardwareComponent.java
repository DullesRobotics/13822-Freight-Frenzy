package org.firstinspires.ftc.teamcode.Hardware;

import org.jetbrains.annotations.Nullable;

/**
 * Acts as a shell around a hardware component to more easily store and reference information about it
 */
public abstract class HardwareComponent {

    /** The hardware this stores */
    protected Object component;
    /** The ID of the motor in the hardwareMap */
    private final String id;
    /** What area this component controls */
    private final HardwareComponentArea componentArea;

    /**
     * Constructs this component without a predefined hardware map entry
     * @param id The id of this component
     * @param componentArea The area of the robot this component controls
     */
    public HardwareComponent(String id, HardwareComponentArea componentArea){
        this.id = id;
        this.componentArea = componentArea;
    }

    /**
     * Constructs this component with a predefined hardware map entry
     * @param id The id of this component
     * @param component The raw hardware reference
     * @param componentArea The area of the robot this component controls
     */
    public HardwareComponent(String id, Object component, HardwareComponentArea componentArea){
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
    public HardwareComponentArea getComponentArea(){
        return componentArea;
    }

    /**
     * Set the hardware this component stores
     * @param component The hardware this stores
     */
    protected void setComponent(Object component){
        this.component = component;
    }

}
