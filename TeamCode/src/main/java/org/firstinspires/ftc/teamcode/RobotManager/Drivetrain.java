package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;

@TargetApi(Build.VERSION_CODES.N)
public class Drivetrain extends Robot {

    private volatile double speed = 0.8f;
    private volatile double minimumPrecisionSpeed = 0.2f;

    /**
     * Initiates drive train motors
     */
    public Drivetrain(LinearOpMode op) {
        super(op);
    }

    /**
     * moves every drive train motor forward
     * @param power The speed to move the motor
     */
    public void setUniformDrivePower(double power){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor -> motor.get().setPower(power));
    }

    /**
     * turns each drive train motor
     * @param power The speed to move the motor
     */
    public void setTurningDrivePower(double power){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor -> motor.get().setPower( motor.isOpposite() ? power : -power ));
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * By pressing the right trigger, you can brake the robot linearly to the minimumSpeed
     * @param c The controller to move the robot with
     */
    public void driveWithController(Controller c){
        addThread(new Thread(() -> {
            while(op.opModeIsActive()){
                /* linear equation to calculate speed based on right trigger's position */
                double currentSpeed = (speed - minimumPrecisionSpeed) * (c.rightTrigger() - 1) + speed;
                getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor ->
                        motor.get().setPower(motor.isOpposite() ? currentSpeed * (-c.rightY() + c.rightX()) : currentSpeed * (-c.rightY() - c.rightX())));
            }
        }), true);
    }

    /**
     * Set the normal speed of the robot in manual control
     * @param speed The speed of the robot in manual control
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Set the lowest speed the robot will move if the trigger is pulled all the way
     * @param minimumPrecisionSpeed The LOWEST speed the robot will go in precision mode.
     */
    public void setMinimumPrecisionSpeed(double minimumPrecisionSpeed) {
        this.minimumPrecisionSpeed = minimumPrecisionSpeed;
    }
}
