package org.firstinspires.ftc.teamcode.RobotManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.TestRobot.RoadRunnerDriveConstants;

//@TargetApi(Build.VERSION_CODES.N)
@Config
public abstract class DriveTrain extends Robot {

    public static double speed = 0.8, minimumPrecisionSpeed = 0.2;
    protected MotorConfiguration autonMotorConfiguration;
    protected final PID pid;

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public DriveTrain(LinearOpMode op, PID pid) {
        super(op);
        this.autonMotorConfiguration = null;
        this.pid = pid;
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
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            motor.get().setPower(power);
    }

    /**
     * turns each drive train motor
     * @param power The speed to move the motor
     */
    public void setTurningDrivePower(double power){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            motor.get().setPower( motor.isOpposite() ? power : -power );
    }

    public void setIndependentDrivePower(double leftPower, double rightPower){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            motor.get().setPower( motor.isOpposite() ? rightPower : leftPower );
    }

    /** Stops and resets the motor, and then reverts state */
    public void resetAllEncoders(){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            motor.stopAndResetEncoder();
    }

    /** Makes all motors run with encoders */
    public void setAllRunWithEncoder(){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            if(motor.getConfiguration().isEncoded())
                motor.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Makes motors run without encoders */
    public void setAllRunWithoutEncoder(){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            if(motor.getConfiguration().isEncoded())
                motor.get().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Makes all motors go into run_to_position mode */
    public void setAllRunToPosition(){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            if(motor.getConfiguration().isEncoded())
                motor.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** @return If any drive train motor is moving */
    public boolean isAnyDriveTrainMotorBusy(){
        for(Motor motor : getMotors(HardwareComponentArea.DRIVE_TRAIN))
            if(motor.get().isBusy())
                return true;
        return false;
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

    /**
     * Returns the PID used by the drivetrain
     */
    public PID getPID(){
        return pid;
    }

    public enum Direction {
        FORWARD, LEFT, RIGHT
    }

}
