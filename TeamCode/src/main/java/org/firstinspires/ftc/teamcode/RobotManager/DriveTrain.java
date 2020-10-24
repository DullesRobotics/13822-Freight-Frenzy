package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor;

@TargetApi(Build.VERSION_CODES.N)
public class DriveTrain extends Robot {

    private volatile double speed = 0.8f;
    private volatile double minimumPrecisionSpeed = 0.2f;

    public DriveTrain(LinearOpMode op) {
        super(op);
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * By pressing the right trigger, you can brake the robot linearly to the minimumSpeed
     * @param c The controller to move the robot with
     */
    public void standardDriveWithController(Controller c){
        addThread(new Thread(() -> {
            double currentSpeed;
            while(op.opModeIsActive()){
                /* linear equation to calculate speed based on right trigger's position */
                currentSpeed = (speed - minimumPrecisionSpeed) * (c.rightTrigger() - 1) + speed;
                for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN)) /* uses regular for-each loop because lambdas require final variables, which is just asking for a heap issue */
                    motor.get().setPower(motor.isOpposite() ? currentSpeed * (-c.rightY() + c.rightX()) : currentSpeed * (-c.rightY() - c.rightX()));
            }
        }), true);
    }

    /**
     * Moves the robot forward a specified number of inches
     * @param inches The number of inches to move
     * @param turn Whether or not to turn
     */
    public void encodedAutoStandardDrive(double inches, boolean turn, boolean turnRight){
        setAllRunToPosition();

        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor ->
                motor.get().setTargetPosition(motor.get().getCurrentPosition() +
                        ((motor.isOpposite() && turnRight) || (!motor.isOpposite() && !turnRight) || !turn ? 1 : -1 ) *
                                (motor.getMotorConfiguration().inchesToCounts(inches))));

        double power = inches < 0 ? -speed : speed;
        setUniformDrivePower(power);

        while(getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().anyMatch(motor -> motor.get().isBusy()))
            setUniformDrivePower(power);

        setUniformDrivePower(0);
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

    /** Stops and resets the motor, and then reverts state */
    public void resetAllEncoders(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(Motor::stopAndResetEncoder);
    }

    /** Makes all motors run with encoders */
    public void setAllRunWithEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(Motor::isEncoded).forEach(motor -> motor.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    /** Makes motors run without encoders */
    public void setAllRunWithoutEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(Motor::isEncoded).forEach(motor -> motor.get().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    /** Makes all motors go into run_to_position mode */
    public void setAllRunToPosition(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(Motor::isEncoded).forEach(motor -> motor.get().setMode(DcMotor.RunMode.RUN_TO_POSITION));
    }
}
