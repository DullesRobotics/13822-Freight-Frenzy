package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.PID;

import java.util.logging.Level;

/**
 * A drive-train for robots that can strafe
 */
@TargetApi(Build.VERSION_CODES.N)
public class MecanumDriveTrain extends StandardDriveTrain {

    private final String flm, frm, blm, brm;

    public MecanumDriveTrain(LinearOpMode op, boolean hasRecorder, String flm, String frm, String blm, String brm) {
        super(op, hasRecorder);
        this.flm = flm;
        this.frm = frm;
        this.blm = blm;
        this.brm = brm;
    }

    /**
     * Driving with the controller, including strafing
     * Braking is NOT dynamic this time, you must hold the bumper
     * TODO make braking toggle?
     * @param c The controller to move the robot with
     */
    @Override
    public void driveWithController(Controller c) {
        getLogger().log(Level.INFO, "Beginning drive with controller, mechanum");
        double staticPrecisionSpeed = minimumPrecisionSpeed + ((speed - minimumPrecisionSpeed) / 4);
        addThread(new Thread(() -> {
            double currentJoystickSpeed = -1;
            getLogger().putData("Joystick Speed", currentJoystickSpeed);
            while(op.opModeIsActive()){
                currentJoystickSpeed = c.rightBumper() ? currentJoystickSpeed : speed;

                double flmPower = (-c.rightY() + c.leftTrigger() - c.rightTrigger()) * currentJoystickSpeed,
                frmPower = (-c.leftY() - c.leftTrigger() + c.rightTrigger()) * currentJoystickSpeed,
                blmPower = (-c.rightY() - c.leftTrigger() + c.rightTrigger()) * currentJoystickSpeed,
                brmPower = (-c.leftY() + c.leftTrigger() - c.rightTrigger()) * currentJoystickSpeed;

                getLogger().putData("Front Power (FLM, FRM)", flmPower + " " + frmPower);
                getLogger().putData("Back Power (BLM, BRM)", blmPower + " " + brmPower);

                setFourWayPower(flmPower, frmPower, blmPower, brmPower);
            }
        }), true, () -> getLogger().clearData());
    }

    /**
     * Has the robot strafe a distance w/o a PID
     * @param distance How far to go in inches. Positive is left, Negative is right
     */
    public void autoStrafeEncoded(double distance){

    }

    /**
     * Strafes the robot for a certain amount of time
     * @param millis The time in milliseconds to strafe
     * @param goLeft If the robot should go left or right
     */
    public void autoStrafeTimed(long millis, boolean goLeft){

    }

    /**
     * Has the robot strafe a set distance using a PID
     * @param distance How far to go in inches. Positive is left, Negative is right
     * @param pid The PID to use
     */
    public void autoStrafeEncodedPID(double distance, PID pid){

    }

    private void setFourWayPower(double flmPower, double frmPower, double blmPower, double brmPower){
        getMotor(flm).get().setPower(flmPower);
        getMotor(frm).get().setPower(frmPower);
        getMotor(blm).get().setPower(blmPower);
        getMotor(brm).get().setPower(brmPower);
    }

}
