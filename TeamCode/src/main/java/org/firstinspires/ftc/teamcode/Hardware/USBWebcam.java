package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

public class USBWebcam extends HardwareComponent {

    /**
     * @param r The op mode this USB webcam is registered in
     * @param id The id of the USB webcam in the hardware map
     */
    public USBWebcam(Robot r, String id) {
        super(r, id, ComponentArea.MISCELLANEOUS);
        r.getLogger().log(Level.INFO, "Adding Webcam: " + id);
        try { setComponent(r.op().hardwareMap.get(WebcamName.class, id));
        } catch (Exception e)
        {
            r.getLogger().log(Level.SEVERE, "Error Adding USB Webcam " + id, e.toString());
            r.op().requestOpModeStop();
        }
    }

    @Override
    public WebcamName get() {
        return (WebcamName) component;
    }

}
