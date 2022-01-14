package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.jetbrains.annotations.Nullable;

import java.util.logging.Level;

/**
 * A drive-train for robots that can strafe
 */
//@TargetApi(Build.VERSION_CODES.N)
public class TankDrive extends StandardDriveTrain {

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public TankDrive(LinearOpMode op, PID pid) {
        super(op, pid);
    }

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public TankDrive(LinearOpMode op) {
        super(op);
    }

    /**
     * Driving with the controller, including strafing
     * Hold either bumper for precision mode
     * @param ctrl The controller to move the robot with
     */
    @Override
    public void driveWithController(Controller ctrl) {
        getLogger().log(Level.INFO, "Beginning drive with controller, mechanum");
        addThread(new Thread(() -> {
            double flmPower, frmPower, blmPower, brmPower, mainPower, currentSpeed, centerSpeed;
            while(op().opModeIsActive()){

                centerSpeed = ctrl.leftTrigger() - ctrl.rightTrigger();
                currentSpeed = ctrl.rightBumper() ? precisionSpeed : speed;
                getLogger().putData("Joystick Speed", currentSpeed);
                getLogger().putData("Motor Speed", currentSpeed);
                setSidedDrivePower(-1 * currentSpeed * ctrl.leftY(), -1 * currentSpeed * ctrl.rightY());
                if(getCenterOmni() != null)
                    getCenterOmni().get().setPower(centerSpeed);

                getLogger().putData("Velocity (FL, FR, BL, BR)", getMotor("FLM").getEncoded().getVelocity() + ", " + getMotor("FRM").getEncoded().getVelocity() + ", " + getMotor("BLM").getEncoded().getVelocity() + ", " + getMotor("BRM").getEncoded().getVelocity());

            }
        }), true, () -> getLogger().clearData());
    }

    /**
     * Strafes the robot for a certain amount of time
     * @param millis The time in milliseconds to strafe
     * @param goLeft If the robot should go left or right
     */
    public void autoStrafeTimed(long millis, boolean goLeft){
        getLogger().log(Level.INFO, "Strafing, Timed");
        long time = System.currentTimeMillis() + millis;
        double tempSpeed = speed;
        while(op().opModeIsActive() && time > System.currentTimeMillis()){
            if(goLeft) tempSpeed *= -1;
            setIndividualDrivePower(speed, -speed, speed, -speed);
            getLogger().putData("Speed (FL, BL, FR, BR)", "(" + speed + ", " + -speed + ", " + -speed + ", " + speed + ")");
        }
        setUniformDrivePower(0);
        getLogger().clearData();
        getLogger().log(Level.INFO, "Finished Strafing, Timed");
    }

    /**
     * @return The center omni wheel shell class or null
     */
    @Nullable
    Motor getCenterOmni(){
        return !getMotors(ComponentArea.CENTER_OMNI).isEmpty() ? getMotors(ComponentArea.CENTER_OMNI).get(0) : null;
    }


}
