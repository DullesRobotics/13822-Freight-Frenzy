package org.firstinspires.ftc.teamcode.HardwareHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Servo extends HardwareComponent<com.qualcomm.robotcore.hardware.Servo> {

    /**
     * @param op The op mode this servo is registered in
     * @param id The id of the servo in the hardware map
     * @param componentArea Where the servo is on the robot
     */
    public Servo(LinearOpMode op, String id, HardwareComponentArea componentArea) {
        super(id, componentArea);
        try { setComponent(op.hardwareMap.servo.get(id));
        } catch (Exception e)
        {
            op.telemetry.addData("Error Adding Servo " + id + ":", e);
            op.telemetry.update();
            op.requestOpModeStop();
        }
    }

}
