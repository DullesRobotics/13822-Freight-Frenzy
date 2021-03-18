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
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.RoadRunnerDriveConstants;

//@TargetApi(Build.VERSION_CODES.N)
@Config
public abstract class DriveTrain extends Robot {

    public static double speed = 0.8, minimumPrecisionSpeed = 0.2;
    protected MotorConfiguration autonMotorConfiguration;
    protected final PID pid;
    protected volatile double maxVel = 30, maxAccel = 30, maxAngularVel = 60, maxAngularAccel = 60;
    /* Distance between two parallel motors in inches */
    protected volatile double trackWidth = 10;
    protected volatile Axis imuAxis = Axis.Z;

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

    /**
     * Set the axis the rev control hub is on
     *                             | Z axis
     *                             |
     *       (Motor Port Side)     |   / X axis
     *                         ____|__/____
     *           Y axis      / *   | /    /|   (IO Side)
     *            _________ /______|/    //      I2C
     *                     /___________ //     Digital
     *                    |____________|/      Analog
     *
     *                   (Servo Port Side)
     *
     *  The positive x axis points toward the USB port(s)
     *
     *  Adjust the axis rotation rate as necessary
     *  Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
     *  flat on a surface
     * @param axis The Axis, X Y or Z
     */
    public void setIMUAxis(Axis axis){
        this.imuAxis = axis;
        adjustRoadRunnerConstants();
    }

    /**
     * Set how far apart two parallel motors are. Default is 10
     * @param inches How fast apart the motors are in inches
     */
    public void setTrackWidth(double inches){
        this.trackWidth = trackWidth;
        adjustRoadRunnerConstants();
    }

    /**
     * Set the autonomous motor configuration variables
     * @param autonMotorConfiguration
     */
    public void setAutonMotorConfiguration(MotorConfiguration autonMotorConfiguration){
        this.autonMotorConfiguration = autonMotorConfiguration;
        adjustRoadRunnerConstants();
    }

    /**
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public void setFeedForwardConstants(double kV, double kA, double kStatic){
        RoadRunnerDriveConstants.kV = kV;
        RoadRunnerDriveConstants.kA = kA;
        RoadRunnerDriveConstants.kStatic = kStatic;
    }

    /**
     * Sets velocity and acceleration constants
     * All variables in unit inches per second (in/s)
     */
    public void setVelocityConstants(double maxVel, double maxAccel, double maxAngularVel, double maxAngularAccel){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxAngularVel = maxAngularVel;
        this.maxAngularVel = maxAngularVel;
        adjustRoadRunnerConstants();
    }

    /**
     * Adjusts DriveConstants to specified ones
     */
    public void adjustRoadRunnerConstants(){
        if(autonMotorConfiguration != null)
        {
            RoadRunnerDriveConstants.TICKS_PER_REV = autonMotorConfiguration.getMotorType().countsPerRev();
            RoadRunnerDriveConstants.MAX_RPM = autonMotorConfiguration.getMotorType().getMaxRPM();
            RoadRunnerDriveConstants.RUN_USING_ENCODER = autonMotorConfiguration.isEncoded();
            RoadRunnerDriveConstants.MOTOR_VELO_PID = new PIDFCoefficients(pid.getKp(), pid.getKi(), pid.getKd(), RoadRunnerDriveConstants.getMotorVelocityF(autonMotorConfiguration.getMotorType().getMaxRPM() / 60 * autonMotorConfiguration.getMotorType().countsPerRev()));
            RoadRunnerDriveConstants.WHEEL_RADIUS = autonMotorConfiguration.getWheelDiameter() / 2;
            RoadRunnerDriveConstants.GEAR_RATIO = autonMotorConfiguration.getGearRatio();
        }
        RoadRunnerDriveConstants.TRACK_WIDTH = trackWidth;
        RoadRunnerDriveConstants.MAX_VEL = maxVel;
        RoadRunnerDriveConstants.MAX_ACCEL = maxAccel;
        RoadRunnerDriveConstants.MAX_ANG_VEL = maxAngularVel;
        RoadRunnerDriveConstants.MAX_ANG_ACCEL = maxAngularAccel;
        RoadRunnerDriveConstants.IMU_AXIS = imuAxis;
    }
}
