package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.PID;

import java.util.logging.Level;

@TargetApi(Build.VERSION_CODES.N)
public class DriveTrain extends Robot {

    protected volatile double speed = 0.8f;
    protected volatile double minimumPrecisionSpeed = 0.2f;

    protected DriveTrain(LinearOpMode op, boolean hasRecorder) {
        super(op, hasRecorder);
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * By pressing the right trigger, you can brake the robot linearly to the minimumSpeed
     * @param c The controller to move the robot with
     */
    public void driveWithController(Controller c){
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
     * Moves the robot forward a specified number of inches
     * @param inches The number of inches to move
     * @param turn Whether or not to turn
     */
    public void encodedAutoDrive(double inches, boolean turn, boolean turnRight){
        setAllRunToPosition();

        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor ->
                motor.get().setTargetPosition(motor.get().getCurrentPosition() +
                        ((motor.isOpposite() && turnRight) || (!motor.isOpposite() && !turnRight) || !turn ? 1 : -1 ) *
                                (motor.getMotorConfiguration().inchesToCounts(inches))));

        double power = inches < 0 ? -speed : speed;
        setUniformDrivePower(power);

        while(op.opModeIsActive() && isAnyMotorBusy())
            setUniformDrivePower(power);

        setUniformDrivePower(0);
    }

    /**
     * Moves the robot for a certain amount of time
     * @param millis The amount of milliseconds to move the robot
     * @param turn If the robot should turn
     * @param turnRight If the robot should turn right
     */
    public void simpleTimedAutoDrive(long millis, boolean turn, boolean turnRight){
        getLogger().log(Level.INFO, "Moving ( turning = " + turn + ", isRight = " + turnRight + ") for " + millis + " milliseconds.");
        long time = System.currentTimeMillis() + millis;
        while(op.opModeIsActive() && time >= System.currentTimeMillis()) {
            getLogger().putData("Time Left", time - System.currentTimeMillis());
            setIndependentDrivePower(turn && !turnRight ? -speed : speed, turn && turnRight ? -speed : speed);
        }
        setUniformDrivePower(0);
        getLogger().clearData();
        getLogger().log(Level.INFO, "Done moving.");
    }

    /**
     * Moves robot forward using a set time and a PID
     * @param millis The amount of milliseconds to move the robot
     * @param pid the PID this motion will use
     * @param tolerance what angle the robot can have moved to be okay
     */
    public void timedAutoDriveForwardPID(long millis, PID pid, double tolerance, boolean forward){
        getLogger().log(Level.INFO, "Driving Forward with PID and Timed");
        long time = System.currentTimeMillis() + millis;
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();
        resetAllEncoders();

        double steer, leftSpeed, rightSpeed, target = imu.getYaw();
        while(op.opModeIsActive() && time >= System.currentTimeMillis()){
            steer = (forward ? 1 : -1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;
            setIndependentDrivePower(leftSpeed, rightSpeed);
            getLogger().putData("Steer", steer);
            getLogger().putData("Time Left", time - System.currentTimeMillis());
            getLogger().putData("Speed (L, R)", "(" + leftSpeed + ", " + rightSpeed + ")");
        }
        getLogger().log(Level.INFO, "Angle to turn: ", target - imu.getYaw());
        op.sleep(400L);
        getLogger().clearData();
        setUniformDrivePower(0);
        getLogger().log(Level.INFO, "Finished Driving Forward");

        /* Correcting angle to make sure it stays in the same line */
        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
        if(op.opModeIsActive() && Math.abs(angleDiff) > 1)
            encodedAutoTurnPID(angleDiff, tolerance, true);
    }

    /**
     * Moves robot forward using encoders and a PID
     * @param inches the distance to move forward in inches
     * @param pid the PID this motion will use
     * @param tolerance what angle the robot can have moved to be okay
     */
    public void encodedAutoStraightPID(double inches, PID pid, double tolerance){
        getLogger().log(Level.INFO, "Driving Forward with PID and Encoded");
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();
        resetAllEncoders();
        setAllRunToPosition();

        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor ->
                motor.get().setTargetPosition(motor.get().getCurrentPosition() +
                        motor.getMotorConfiguration().inchesToCounts(inches)));

        double steer, leftSpeed, rightSpeed, target = imu.getYaw();
        while(op.opModeIsActive() && isAnyMotorBusy()){
            steer = (inches < 0 ? -1 : 1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;
            setIndependentDrivePower(leftSpeed, rightSpeed);

            getLogger().putData("Steer", steer);
            if(getMotors(HardwareComponentArea.DRIVE_TRAIN).size() > 0) {
                getLogger().putData("Target Ticks", getMotors(HardwareComponentArea.DRIVE_TRAIN).get(0).get().getTargetPosition());
                getLogger().putData("Current Ticks", steer);
            }
            getLogger().putData("Speed (L, R)", "(" + leftSpeed + ", " + rightSpeed + ")");
        }
        getLogger().log(Level.INFO, "Angle to turn: ", target - imu.getYaw());
        op.sleep(400L);
        setUniformDrivePower(0);
        setAllRunWithEncoder();
        getLogger().clearData();
        getLogger().log(Level.INFO, "Finished Driving Forward");

        /* Correcting angle to make sure it stays in the same line */
        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
        if(op.opModeIsActive() && Math.abs(angleDiff) > 1)
            encodedAutoTurnPID(angleDiff, tolerance, true);
    }


    /**
     * Turns robot a certain angle with the PID
     * @param angle The angle to move the robot
     * @param tolerance Where the robot doesn't have to correct further
     * @param correction If this is a small correction, uses a smaller speed
     */
    public void encodedAutoTurnPID(double angle, double tolerance, boolean correction) {
        getLogger().log(Level.INFO, "Turning with PID, Correction:" + correction);
        double tempSpeed = correction ? 0.2 : speed;
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();
        double startYaw = imu.getYaw(), targetYaw = imu.getYaw() + angle;

        /* Correcting the angle just in case */
        targetYaw = targetYaw > 360 ? targetYaw % 360 : targetYaw;
        targetYaw = targetYaw < 0 ? targetYaw + 360 : targetYaw;

        double distanceToTarget = IMU.distanceBetweenAngles(startYaw, targetYaw);

        for (int i = 1; i <= 8; i++) {
            while (op.opModeIsActive() && (distanceToTarget < tolerance) ^ (distanceToTarget > -tolerance)) {
                distanceToTarget = IMU.distanceBetweenAngles(startYaw, targetYaw);
                if (distanceToTarget < 0 + tolerance)
                    if (angle > 0)
                        setTurningDrivePower(tempSpeed / i);
                    else
                        setTurningDrivePower(tempSpeed / i);
                else if (distanceToTarget > 0 - tolerance)
                    if (angle > 0)
                        setTurningDrivePower(tempSpeed / i);
                    else
                        setTurningDrivePower(-tempSpeed / i);
                else {
                    setUniformDrivePower(0);
                    break;
                }
                getLogger().putData("Distance to target", distanceToTarget);
                getLogger().putData("Target Yaw", targetYaw);
                getLogger().putData("Start Yaw", startYaw);
                getLogger().putData("Angle", angle);
                getLogger().putData("Iteration", i);
            }
            op.sleep(200);
        }
        getLogger().clearData();
        getLogger().log(Level.INFO, "PID Done Turning");
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

    public void setIndependentDrivePower(double leftPower, double rightPower){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(motor -> motor.get().setPower( motor.isOpposite() ? rightPower : leftPower ));
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
        getMotors(HardwareComponentArea.DRIVE_TRAIN).forEach(Motor::stopAndResetEncoder);
    }

    /** Makes all motors run with encoders */
    public void setAllRunWithEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(m -> m.getMotorConfiguration().isEncoded()).forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    /** Makes motors run without encoders */
    public void setAllRunWithoutEncoder(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(m -> m.getMotorConfiguration().isEncoded()).forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    /** Makes all motors go into run_to_position mode */
    public void setAllRunToPosition(){
        getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().filter(m -> m.getMotorConfiguration().isEncoded()).forEach(m -> m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION));
    }

    /** @return If any drive train motor is moving */
    public boolean isAnyMotorBusy(){
        return getMotors(HardwareComponentArea.DRIVE_TRAIN).stream().allMatch(motor -> motor.get().isBusy());
    }
}
