package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorConfiguration;
import org.firstinspires.ftc.teamcode.Libraries.PID;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.DriveConstants;

@TargetApi(Build.VERSION_CODES.N)
public abstract class DriveTrain extends Robot {

    protected volatile double speed = 0.8f, minimumPrecisionSpeed = 0.2f;
    protected MotorConfiguration autonMotorConfiguration;
    protected final PID pid;
    protected volatile double maxVel = 30, maxAccel = 30, maxAngularVel = 60, maxAngularAccel = 60;
    /* Distance between two parallel motors in inches */
    protected volatile double trackWidth = 10;
    protected volatile Axis imuAxis = Axis.Z;

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     * @param hardwareComponents The list of hardware used by the robot
     */
    public DriveTrain(LinearOpMode op, HardwareComponent[] hardwareComponents, PID pid) {
        super(op, hardwareComponents);
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
        DriveConstants.kV = kV;
        DriveConstants.kA = kA;
        DriveConstants.kStatic = kStatic;
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
            DriveConstants.TICKS_PER_REV = autonMotorConfiguration.getMotorType().countsPerRev();
            DriveConstants.MAX_RPM = autonMotorConfiguration.getMaxRPM();
            DriveConstants.RUN_USING_ENCODER = autonMotorConfiguration.isEncoded();
            DriveConstants.MOTOR_VELO_PID = new PIDFCoefficients(pid.getKp(), pid.getKi(), pid.getKd(), DriveConstants.getMotorVelocityF(autonMotorConfiguration.getMaxRPM() / 60 * autonMotorConfiguration.getMotorType().countsPerRev()));
            DriveConstants.WHEEL_RADIUS = autonMotorConfiguration.getWheelDiameter() / 2;
            DriveConstants.GEAR_RATIO = autonMotorConfiguration.getGearRatio();
        }
        DriveConstants.TRACK_WIDTH = trackWidth;
        DriveConstants.MAX_VEL = maxVel;
        DriveConstants.MAX_ACCEL = maxAccel;
        DriveConstants.MAX_ANG_VEL = maxAngularVel;
        DriveConstants.MAX_ANG_ACCEL = maxAngularAccel;
        DriveConstants.IMU_AXIS = imuAxis;
    }
}
