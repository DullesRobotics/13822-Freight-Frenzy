package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOn;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnType;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.EasyOpenCV;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.RobotRecorder;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.Vuforia;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.logging.Level;

public class AddOnHandler {

    private HashMap<AddOnType, AddOn> addOns = new HashMap<>();
    private Robot r;

    public AddOnHandler (Robot r){ this.r = r; }

    /**
     * Initializes the add-on
     * @param ao
     */
    public void initAddOn(@NotNull AddOn ao){
        if(addOns.containsKey(ao.getType()))
            addOns.get(ao.getType()).stop();
        ao.init(r);
        addOns.put(ao.getType(), ao);
    }

    /**
     * Initializes and starts the add-on
     * @param ao
     */
    public void initAndStartAddOn(@NotNull AddOn ao){
        if(addOns.containsKey(ao.getType()))
            addOns.get(ao.getType()).stop();
        addOns.put(ao.getType(), ao);
        addOns.get(ao.getType()).init(r);
        addOns.get(ao.getType()).start();
    }

    /**
     * Starts the add-on with that type
     * @param addonType The type of add-on to start
     * @return If successful
     */
    public boolean startAddOn(@NotNull AddOnType addonType){
        if(addOns.containsKey(addonType))
            addOns.get(addonType).start();
        return addOns.containsKey(addonType);
    }

    /**
     * Stops the add-on with that type
     * @param addonType The type of add-on to stop
     */
    public void stopAddOn(@NotNull AddOnType addonType){
        if(addOns.containsKey(addonType))
            addOns.get(addonType).stop();
    }

    /**
     * Accesses an add-on, if it exists
     * @param addon The add-on to access
     * @return The add-on if it exists, otherwise null
     */
    @Nullable
    public AddOn getAddOn(@NotNull AddOnType addon){
        return addOns.containsKey(addon) ? addOns.get(addon) : null;
    }

    /**
     * Deletes an add-on from this robot.
     * @param addon The add-on type to delete
     */
    public void deleteAddOn(@NotNull AddOnType addon){
        if(addOns.containsKey(addon))
            addOns.get(addon).stop();
        addOns.remove(addon);
    }

}
