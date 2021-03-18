package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

/**
 * An external addition that can be added to the robot class to perform further action
 * This class acts as a barrier to functions performed to ensure nothing bad happens
 */
public abstract class AddOn {

    private AddOnType type;
    private boolean isRunning = false;
    private boolean isInitialized = false;
    private volatile Robot r;

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
        if(r != null)
            r.getLogger().log(Level.INFO, "Starting addon " + getType().toString() + " ...");
        startAO();
        if(r != null)
            r.getLogger().log(Level.INFO, "Started addon " + getType().toString() + " ...");
    }

    /**
     * Stops the add-on
     */
    protected void stop(){
        if(!isRunning || !isInitialized) return;
        isRunning = false;
        isInitialized = false;
        if(r != null)
            r.getLogger().log(Level.INFO, "Stopping addon " + getType().toString() + " ...");
        stopAO();
        if(r != null)
            r.getLogger().log(Level.INFO, "Stopped addon " + getType().toString());
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
        this.r = r;
        if(r != null)
            r.getLogger().log(Level.INFO, "Initializing addon " + getType().toString() + " ...");
        initAO(r);
        if(r != null)
            r.getLogger().log(Level.INFO, "Initialized addon " + getType().toString());
        isInitialized = true;
    }

    /** Initializes the actual add-on */
    protected abstract void initAO(Robot r);

    /** Starts the actual add-on */
    protected abstract void startAO();

    /** Stops the actual add-on */
    protected abstract void stopAO();

}
