package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;

@TargetApi(Build.VERSION_CODES.N)
public abstract class DriveTrain extends Robot {

    protected volatile double speed = 0.8f;
    protected volatile double minimumPrecisionSpeed = 0.2f;

    protected DriveTrain(LinearOpMode op, HardwareComponent[] hardwareComponents) {
        super(op, hardwareComponents);
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * @param c The controller to move the robot with
     */
    public abstract void driveWithController(Controller c);

    /**
     * moves every drive train motor forward
     * @param power The speed to move the motor
     */
    public void setUniformDrivePower(double power){
        getMotors(HardwareComponentArea.DRIVE_TRAIN)
                .forEach(motor -> motor.get().setPower(power));
    }

    /**
     * turns each drive train motor
     * @param power The speed to move the motor
     */
    public void setTurningDrivePower(double power){
        getMotors(HardwareComponentArea.DRIVE_TRAIN)
                .forEach(motor -> motor.get().setPower( motor.isOpposite() ? power : -power ));
    }

    public void setIndependentDrivePower(double leftPower, double rightPower){
        getMotors(HardwareComponentArea.DRIVE_TRAIN)
                .forEach(motor -> motor.get().setPower( motor.isOpposite() ? rightPower : leftPower ));
    }

    /**
     * Set the normal speed of the robot in manual control
     * @param speed The speed of the robot in manual control
     */
    public void setRobotBaseSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Set the lowest speed the robot will move if the trigger is pulled all the way
     * @param minimumPrecisionSpeed The LOWEST speed the robot will go in precision mode.
     */
    public void setMinimumPrecisionSpeed(double minimumPrecisionSpeed) {
        this.minimumPrecisionSpeed = minimumPrecisionSpeed;
    }

    /** Stops and resets the motor, and then reverts state */
    public void resetAllEncoders(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN)
                .forEach(Motor::stopAndResetEncoder);
    }

    /** Makes all motors run with encoders */
    public void setAllRunWithEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream()
                .filter(m -> m.getConfiguration().isEncoded())
                .forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    /** Makes motors run without encoders */
    public void setAllRunWithoutEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream()
                .filter(m -> m.getConfiguration().isEncoded())
                .forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    /** Makes all motors go into run_to_position mode */
    public void setAllRunToPosition(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream()
                .filter(m -> m.getConfiguration().isEncoded())
                .forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION));
    }

    /** @return If any drive train motor is moving */
    public boolean isAnyDriveTrainMotorBusy(){
        return getMotors(HardwareComponentArea.DRIVE_TRAIN).stream()
                .allMatch(motor -> motor.get().isBusy());
    }

    /**
     * A way to stop the robot for x amount of milliseconds
     * @param milliseconds The amount of milliseconds to pause for
     */
    public void autonWait(long milliseconds){
        long timeToUnpause = System.currentTimeMillis() + milliseconds;
        while(op().opModeIsActive() && System.currentTimeMillis() < timeToUnpause)
            getLogger().putData("Pause Time Left", timeToUnpause - System.currentTimeMillis());
        getLogger().removeData("Pause Time Left");
    }
}
