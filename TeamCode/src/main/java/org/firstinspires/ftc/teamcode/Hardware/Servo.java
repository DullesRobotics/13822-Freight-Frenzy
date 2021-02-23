package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.logging.Level;

public class Servo extends HardwareComponent {

    /**
     * @param r The op mode this servo is registered in
     * @param id The id of the servo in the hardware map
     * @param componentArea Where the servo is on the robot
     */
    public Servo(Robot r, String id, HardwareComponentArea componentArea) {
        super(r, id, componentArea);
        try { setComponent(r.op().hardwareMap.servo.get(id));
        } catch (Exception e)
        {
            r.getLogger().logKeyed(Level.SEVERE, "Error Adding Servo " + id, e.toString());
            r.op().requestOpModeStop();
        }
    }

    @Override
    public com.qualcomm.robotcore.hardware.Servo get() {
        return (com.qualcomm.robotcore.hardware.Servo) component;
    }

}
