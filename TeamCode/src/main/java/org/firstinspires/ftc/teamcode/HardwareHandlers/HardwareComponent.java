package org.firstinspires.ftc.teamcode.HardwareHandlers;

/**
 * Acts as a shell around a hardware component to more easily store and reference information about it
 * @param <E> The type of hardware this stores
 */
public class HardwareComponent<E> {

    /** The hardware this stores */
    private E component;
    /** The ID of the motor in the hardwareMap */
    private final String id;
    /** What area this component controls */
    private final HardwareComponentArea componentArea;

    /**
     * Constructs this component without a predefined hardware map entry
     * @param id The id of this component
     * @param componentArea The area of the robot this component controlls
     */
    public HardwareComponent(String id, HardwareComponentArea componentArea){
        this.id = id;
        this.componentArea = componentArea;
    }

    /**
     * Constructs this component with a predefined hardware map entry
     * @param id The id of this component
     * @param component The raw hardware reference
     * @param componentArea The area of the robot this component controlls
     */
    public HardwareComponent(String id, E component, HardwareComponentArea componentArea){
        this.id = id;
        this.component = component;
        this.componentArea = componentArea;
    }

    /**
     * Gets the raw component type of the hardware. <br\>
     * e.g. DCMotor for Motor
     * @return The raw component type
     */
    public E get() { return component; }

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
    protected void setComponent(E component){
        this.component = component;
    }

}
