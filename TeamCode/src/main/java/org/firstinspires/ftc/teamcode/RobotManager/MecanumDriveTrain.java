package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.PID;

/**
 * A drive-train for robots that can strafe
 */
public class MecanumDriveTrain extends DriveTrain {

    public MecanumDriveTrain(LinearOpMode op, boolean hasRecorder) {
        super(op, hasRecorder);
    }

    /**
     * Driving with the controller, including strafing
     * @param c The controller to move the robot with
     */
    @Override
    public void driveWithController(Controller c) {
        addThread(new Thread(() -> {
            double currentSpeed;
            while(op.opModeIsActive()){
                /* linear equation to calculate speed based on right trigger's position */
                currentSpeed = (speed - minimumPrecisionSpeed) * (c.rightTrigger() - 1) + speed;
                for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN)) /* uses regular for-each loop because lambdas require final variables, which is just asking for a heap issue */
                    motor.get().setPower(motor.isOpposite() ? currentSpeed * (-c.rightX() + c.rightX()) : currentSpeed * (-c.rightY() - c.rightX()));
            }
        }), true);
    }

    /**
     * Has the robot strafe a distance w/o a PID
     * @param distance How far to go in inches. Positive is left, Negative is right
     */
    public void encodedAutoStrafe(double distance){

    }

    /**
     * Strafes the robot for a certain amount of time
     * @param millis The time in milliseconds to strafe
     * @param goLeft If the robot should go left or right
     */
    public void timedAutoStrafe(long millis, boolean goLeft){

    }

    /**
     * Has the robot strafe a set distance using a PID
     * @param distance How far to go in inches. Positive is left, Negative is right
     * @param pid The PID to use
     */
    public void encodedAutoStrafePID(double distance, PID pid){

    }

}
