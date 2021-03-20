package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

public class TouchSensor extends HardwareComponent {

    /**
     * @param r The op mode this servo is registered in
     * @param id The id of the servo in the hardware map
     * @param componentArea Where the servo is on the robot
     */
    public TouchSensor(Robot r, String id, ComponentArea componentArea) {
        super(r, id, componentArea);
        r.getLogger().log(Level.INFO, "Adding Touch Sensor: " + id);
        try { setComponent(r.op().hardwareMap.touchSensor.get(id));
        } catch (Exception e)
        {
            r.getLogger().log(Level.SEVERE, "Error Adding Touch Sensor " + id, e.toString());
            r.op().requestOpModeStop();
        }
    }

    @Override
    public com.qualcomm.robotcore.hardware.TouchSensor get() {
        return (com.qualcomm.robotcore.hardware.TouchSensor) component;
    }

}
