package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.PID;

import java.util.logging.Level;

/**
 * A drive-train for robots that can strafe
 */
@TargetApi(Build.VERSION_CODES.N)
public class MecanumDriveTrain extends StandardDriveTrain {

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public MecanumDriveTrain(LinearOpMode op) {
        super(op);
    }

    /**
     * Driving with the controller, including strafing
     * Braking is NOT dynamic this time, you must hold the bumper
     * TODO make braking toggle?
     * TODO is there any way to make braking dynamic?
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
                currentJoystickSpeed = c.rightBumper() ? staticPrecisionSpeed : speed;

                double originalPower = (-c.rightY() + c.leftTrigger() - c.rightTrigger()) * currentJoystickSpeed,
                oppositePower = (-c.leftY() - c.leftTrigger() + c.rightTrigger()) * currentJoystickSpeed;

                getLogger().putData("Front Power (FLM, FRM)", originalPower + " " + oppositePower);
                getLogger().putData("Back Power (BLM, BRM)", oppositePower + " " + originalPower);

                setMechanumPower2D(originalPower, oppositePower);
            }
        }), true, () -> getLogger().clearData());
    }

    /**
     * Has the robot strafe a distance w/o a PID
     * @param inches How far to go in inches. Positive is left, Negative is right
     */
    public void autoStrafeEncoded(double inches){
        getLogger().log(Level.INFO, "Strafing with Encoders");
        resetAllEncoders();
        setAllRunToPosition();

        double power = inches < 0 ? -speed : speed;

        /*
         *       FRONT            BACK
         * LEFT  !opp !strafeOpp  !opp strafeOpp
         * RIGHT opp  strafeOpp   opp  !strafeOpp
         *
         * Loops through every motor and sets the position for it to go to. If it's a front motor is subtracts the ticks, otherwise it adds
         */
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().forEach(motor -> {
            /* if is front motor */
            boolean isFront = (motor.isOpposite() && motor.isStrafeOpposite()) || (!motor.isOpposite() && !motor.isStrafeOpposite());
            motor.get().setTargetPosition(motor.get().getCurrentPosition() + (isFront ? -1 : 1) * motor.getConfiguration().inchesToCounts(inches));
        });

        setUniformDrivePower(power);
        while(op.opModeIsActive() && isAnyDriveTrainMotorBusy()) {
            getLogger().putData("Power", power);
            setUniformDrivePower(power);
        }
        getLogger().clearData();
        setUniformDrivePower(0);
        getLogger().log(Level.INFO, "Finished strafing with Encoders");
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
        while(op.opModeIsActive() && time > System.currentTimeMillis()){
            if(goLeft) tempSpeed *= -1;
            setMechanumPower4D(speed, -speed, speed, -speed);
            getLogger().putData("Speed (FL, BL, FR, BR)", "(" + speed + ", " + -speed + ", " + -speed + ", " + speed + ")");
        }
        setUniformDrivePower(0);
        getLogger().clearData();
        getLogger().log(Level.INFO, "Finished Strafing, Timed");
    }

    /**
     * Strafes the robot for a certain amount of time
     * @param millis The time in milliseconds to strafe
     * @param goLeft If the robot should go left or right
     * @param pid The PID to use
     * @param tolerance How far apart (angle in degrees) it's okay to be off from straight
     */
    public void autoStrafeTimedPID(long millis, boolean goLeft, PID pid, double tolerance){
        getLogger().log(Level.INFO, "Strafing with a PID, Timed");
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();

        long time = System.currentTimeMillis() + millis;
        double steer, leftSpeed, rightSpeed, target = imu.getYaw();

        while(op.opModeIsActive() && time > System.currentTimeMillis()){
            steer = (goLeft ? 1 : -1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            setMechanumPower4D(leftSpeed, -leftSpeed, rightSpeed, -rightSpeed);

            getLogger().putData("Steer", steer);
            getLogger().putData("Speed (FL, BL, FR, BR)", "(" + leftSpeed + ", " + -leftSpeed + ", " + -rightSpeed + ", " + rightSpeed + ")");
        }
        getLogger().log(Level.INFO, "Angle to turn", target - imu.getYaw());
        setUniformDrivePower(0);
        setAllRunWithEncoder();
        getLogger().clearData();
        getLogger().log(Level.INFO, "Finished Strafing w/ PID, Timed");

        /* Correcting angle to make sure it stays in the same line */
        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
        if(op.opModeIsActive() && Math.abs(angleDiff) > tolerance)
            autoTurnPID(angleDiff, tolerance, true);
    }

    /**
     * Has the robot strafe a set distance using a PID
     * @param inches How far to go in inches. Positive is left, Negative is right
     * @param pid The PID to use
     * @param tolerance How far apart (angle in degrees) it's okay to be off from straight
     */
    public void autoStrafeEncodedPID(double inches, PID pid, double tolerance){
        getLogger().log(Level.INFO, "Strafing with a PID and Encoders");
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();
        resetAllEncoders();
        setAllRunToPosition();

        /* Loops through every motor and sets the position for it to go to. If it's strafe opposite (front right, back left), it subtracts the ticks. Otherwise it adds them. */
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().forEach(motor ->
                motor.get().setTargetPosition(motor.get().getCurrentPosition() + (motor.isStrafeOpposite() ? -1 : 1) * motor.getConfiguration().inchesToCounts(inches)));

        double steer, leftSpeed, rightSpeed, target = imu.getYaw();

        while(op.opModeIsActive() && isAnyDriveTrainMotorBusy()){
            steer = (inches < 0 ? -1 : 1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            setMechanumPower4D(leftSpeed, -leftSpeed, rightSpeed, -rightSpeed);

            getLogger().putData("Steer", steer);
            if(getMotors(HardwareComponentArea.DRIVE_TRAIN).size() > 0) {
                /* TODO make this more clear, not sure WHICH motor has its ticks recorded */
                getLogger().putData("Target Ticks", getMotors(HardwareComponentArea.DRIVE_TRAIN).get(0).get().getTargetPosition());
                getLogger().putData("Current Ticks", steer);
            }
            getLogger().putData("Speed (FL, BL, FR, BR)", "(" + leftSpeed + ", " + -leftSpeed + ", " + -rightSpeed + ", " + rightSpeed + ")");
        }
        getLogger().log(Level.INFO, "Angle to turn", target - imu.getYaw());
        setUniformDrivePower(0);
        setAllRunWithEncoder();
        getLogger().clearData();
        getLogger().log(Level.INFO, "Finished Strafing, w/ PID, Encoded");

        /* Correcting angle to make sure it stays in the same line */
        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
        if(op.opModeIsActive() && Math.abs(angleDiff) > tolerance)
            autoTurnPID(angleDiff, tolerance, true);
    }

    /**
     * Sets the power for mechanum motors and the opposite ones
     * @param original The original power
     * @param opposite The opposite power
     */
    private void setMechanumPower2D(double original, double opposite){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().forEach(motor ->
            motor.get().setPower(motor.isStrafeOpposite() ? opposite : original));
    }

    /**
     * Sets power for each kind of drive-train motor
     * @param originalLeft The power for the "original" left motor (usually front left)
     * @param oppositeLeft The power for the "opposite" left motor (usually back left)
     * @param originalRight The power for the "original" right motor (usually back right)
     * @param oppositeRight The power for the "opposite" right motor (usually front right)
     */
    private void setMechanumPower4D(double originalLeft, double oppositeLeft, double originalRight, double oppositeRight){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().forEach(motor ->
            motor.get().setPower(!motor.isOpposite() ? !motor.isStrafeOpposite() ? originalLeft : oppositeLeft : !motor.isStrafeOpposite() ? originalRight : oppositeRight));
    }

    /**
     * Sets power for the front and back motors
     * @param frontPower The front motor power
     * @param backPower The back motor power
     */
    private void setMechanumPowerFB(double frontPower, double backPower){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().forEach(motor -> {
            boolean isFront = (motor.isOpposite() && motor.isStrafeOpposite()) || (!motor.isOpposite() && !motor.isStrafeOpposite());
            motor.get().setPower(isFront ? frontPower : backPower);
        });
    }

}
