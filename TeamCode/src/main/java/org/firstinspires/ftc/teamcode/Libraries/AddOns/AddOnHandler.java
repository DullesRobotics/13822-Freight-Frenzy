package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;

public class AddOnHandler {

    private HashMap<AddOns, AddOn> addOns = new HashMap<>();
    private Robot r;

    public AddOnHandler (Robot r){ this.r = r; }

    /**
     * Initializes and starts an add-on for this robot.
     * @param addon The add-on type to enable
     * @param start If the add-on should be started immediately
     */
    public void initAddOn(@NotNull AddOns addon, boolean start, Object... parameters){
        AddOn ao;
        switch(addon){
            case ROBOT_RECORDER: ao = new RobotRecorder(r); break;
            case VUFORIA: ao = new Vuforia(r, parameters); break;
            case OPEN_CV: ao = new OpenCV(r, parameters);
            default: return;
        }
        if(start) ao.start();
        addOns.put(addon, ao);
    }

    /**
     * Accesses an add-on, if it exists
     * @param addon The add-on to access
     * @return The add-on if it exists, otherwise null
     */
    @Nullable
    public AddOn getAddOn(@NotNull AddOns addon){
        return addOns.containsKey(addon) ? addOns.get(addon) : null;
    }

    /**
     * Deletes an add-on from this robot.
     * @param addon The add-on type to delete
     */
    public void deleteAddOn(@NotNull AddOns addon){
        if(addOns.containsKey(addon))
            addOns.get(addon).stop();
        addOns.remove(addon);
    }

}
