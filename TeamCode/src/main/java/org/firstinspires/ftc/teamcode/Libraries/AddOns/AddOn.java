package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;

/**
 * An external addition that can be added to the robot class to perform further action
 * This class acts as a barrier to functions performed to ensure nothing bad happens
 */
public abstract class AddOn {

    private AddOnType type;
    private boolean isRunning = false;
    private boolean isInitialized = false;

    /**
     * Passes up add-on var
     * @param type The Type of Add-On. Ensures only one exists
     */
    AddOn (AddOnType type) {
        this.type = type;
    }

    /** If the add-on is running */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Starts the add-on
     */
    protected void start(){
        if(isRunning || !isInitialized) return;
        isRunning = true;
        startAO();
    }

    /**
     * Stops the add-on
     */
    protected void stop(){
        if(!isRunning) return;
        isRunning = false;
        stopAO();
    }

    /**
     * @return The type of add-on this is.
     */
    public AddOnType getType(){
        return type;
    }

    /**
     * Initializes the add-on
     */
    public void init(Robot r){
        initAO(r);
        isInitialized = true;
    }

    /** Initializes the actual add-on */
    protected abstract void initAO(Robot r);

    /** Starts the actual add-on */
    protected abstract void startAO();

    /** Stops the actual add-on */
    protected abstract void stopAO();

}
