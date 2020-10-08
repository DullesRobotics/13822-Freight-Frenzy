package org.firstinspires.ftc.teamcode.HardwareHandlers.motor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareHandlers.HardwareComponent;
import org.firstinspires.ftc.teamcode.HardwareHandlers.HardwareComponentArea;

/**
 * Acts as a shell around DCMotor, providing more information and automating more processes surrounding it.
 */
public class Motor extends HardwareComponent<DcMotor> {

    /**
     * @param op The op mode this motor is registered in
     * @param id The id of the motor in the hardware map
     * @param componentArea Where the motor is on the robot
     */
    public Motor(LinearOpMode op, String id, HardwareComponentArea componentArea)
    {
        super(id, componentArea);
        try {
            setComponent(op.hardwareMap.dcMotor.get(id));
            op.hardwareMap.dcMotor.get(id).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            op.telemetry.addData("Error Adding Motor " + id + ":", e);
            op.telemetry.update();
            op.requestOpModeStop();
        }
    }
}
