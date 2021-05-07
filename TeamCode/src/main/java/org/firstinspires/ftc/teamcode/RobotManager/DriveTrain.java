package org.firstinspires.ftc.teamcode.RobotManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Libraries.PID;

//@TargetApi(Build.VERSION_CODES.N)
@Config
public abstract class DriveTrain extends Robot {

    public static double speed = 1, precisionSpeed = 0.2;
    protected MotorConfiguration autonMotorConfiguration;
    protected PID pid = null;

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
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public DriveTrain(LinearOpMode op) {
        super(op);
        this.autonMotorConfiguration = null;
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * @param c The controller to move the robot with
     */
    public abstract void driveWithController(Controller c);

    /**
     * moves every drive train motor forward/backward
     * @param power The speed to move the motors
     */
    public void setUniformDrivePower(double power){
        for(DrivetrainMotor motor : getDrivetrainMotors())
            if(motor.get() != null)
                motor.get().setPower(power);
    }

    /**
     * Sets the power of each individual motor
     * @param leftSidePower Left motors' power
     * @param rightSidePower right motors' power
     */
    public void setSidedDrivePower(double leftSidePower, double rightSidePower) {
        DrivetrainMotor flm = getDrivetrainMotor(MotorType.DrivetrainPosition.FLM);
        DrivetrainMotor frm = getDrivetrainMotor(MotorType.DrivetrainPosition.FRM);
        DrivetrainMotor blm = getDrivetrainMotor(MotorType.DrivetrainPosition.BLM);
        DrivetrainMotor brm = getDrivetrainMotor(MotorType.DrivetrainPosition.BRM);
        if(flm != null && flm.get() != null) flm.get().setPower(leftSidePower);
        if(frm != null && frm.get() != null) frm.get().setPower(rightSidePower);
        if(blm != null && blm.get() != null) blm.get().setPower(leftSidePower);
        if(brm != null && brm.get() != null) brm.get().setPower(rightSidePower);
    }

    /**
     * Sets the power of each individual motor
     * @param flmPower Front left motor power
     * @param frmPower Front right motor power
     * @param blmPower Back left motor power
     * @param brmPower Back right motor power
     */
    public void setIndividualDrivePower(double flmPower, double frmPower, double blmPower, double brmPower) {
        DrivetrainMotor flm = getDrivetrainMotor(MotorType.DrivetrainPosition.FLM);
        DrivetrainMotor frm = getDrivetrainMotor(MotorType.DrivetrainPosition.FRM);
        DrivetrainMotor blm = getDrivetrainMotor(MotorType.DrivetrainPosition.BLM);
        DrivetrainMotor brm = getDrivetrainMotor(MotorType.DrivetrainPosition.BRM);
        if(flm != null && flm.get() != null) flm.get().setPower(flmPower);
        if(frm != null && frm.get() != null) frm.get().setPower(frmPower);
        if(blm != null && blm.get() != null) blm.get().setPower(blmPower);
        if(brm != null && brm.get() != null) brm.get().setPower(brmPower);
    }

    /**
     * turns each drive train motor
     * @param power The speed to move the motor
     */
    public void setTurningDrivePower(double power){
        for(Motor motor : getDrivetrainMotors())
            if(motor.get() != null)
                motor.get().setPower( motor.isFlipped() ? power : -power );
    }

    /** Stops and resets the motor, and then reverts state */
    public void resetAllEncoders(){
        for(Motor motor : getDrivetrainMotors())
            motor.stopAndResetEncoder();
    }

    /** Makes all motors run with encoders */
    public void setAllRunWithEncoder(){
        for(Motor motor : getDrivetrainMotors())
            if(motor.isEncoded() && motor.get() != null)
                motor.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Makes motors run without encoders */
    public void setAllRunWithoutEncoder(){
        for(Motor motor : getDrivetrainMotors())
            if(motor.isEncoded() && motor.get() != null)
                motor.get().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Makes all motors go into run_to_position mode */
    public void setAllRunToPosition(){
        for(Motor motor : getDrivetrainMotors())
            if(motor.isEncoded() && motor.get() != null)
                motor.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** @return If any drive train motor is moving */
    public boolean isAnyDriveTrainMotorBusy(){
        for(Motor motor : getDrivetrainMotors())
            if(motor.get() != null && motor.get().isBusy())
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
