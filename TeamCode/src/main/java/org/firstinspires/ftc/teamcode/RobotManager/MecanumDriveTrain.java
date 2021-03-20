package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.PID;

import java.util.logging.Level;

/**
 * A drive-train for robots that can strafe
 */
//@TargetApi(Build.VERSION_CODES.N)
public class MecanumDriveTrain extends StandardDriveTrain {

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public MecanumDriveTrain(LinearOpMode op, PID pid) {
        super(op, pid);
    }

    /**
     * Driving with the controller, including strafing
     * Braking is NOT dynamic this time, you must hold the bumper
     * TODO make braking toggle?
     * TODO is there any way to make braking dynamic?
     * @param c The controller to move the robot with
     */
    @Override
    public void driveWithController(Controller ctrl) {
        getLogger().log(Level.INFO, "Beginning drive with controller, mechanum");
        double staticPrecisionSpeed = minimumPrecisionSpeed + ((speed - minimumPrecisionSpeed) / 4);
        addThread(new Thread(() -> {
            double flmPower, frmPower, blmPower, brmPower, currentSpeed;
            while(op().opModeIsActive()){
                currentSpeed = ctrl.rightBumper() ? staticPrecisionSpeed : speed;
                getLogger().putData("Joystick Speed", currentSpeed);

                flmPower = (-ctrl.rightY() + ctrl.leftTrigger() - ctrl.rightTrigger());
                frmPower = (-ctrl.leftY() - ctrl.leftTrigger() + ctrl.rightTrigger());
                blmPower = (-ctrl.rightY() - ctrl.leftTrigger() + ctrl.rightTrigger());
                brmPower = (-ctrl.leftY() + ctrl.leftTrigger() - ctrl.rightTrigger());

                getLogger().putData("Set Power (FL, FR, BL, BR)", flmPower + ", " + frmPower + ", " + blmPower + ", " + brmPower);
                getLogger().putData("Power (FL, FR, BL, BR)", getMotor("FLM").get().getPower() + ", " + getMotor("FRM").get().getPower() + ", " + getMotor("BLM").get().getPower() + ", " + getMotor("BRM").get().getPower());
                getLogger().putData("Velocity (FL, FR, BL, BR):", getMotor("FLM").getEncoded().getVelocity() + ", " + getMotor("FRM").getEncoded().getVelocity() + ", " + getMotor("BLM").getEncoded().getVelocity() + ", " + getMotor("BRM").getEncoded().getVelocity());

                setIndividualDrivePower(flmPower, frmPower, blmPower, brmPower);
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
        for(DrivetrainMotor motor : getDrivetrainMotors()){
            /* if is front motor */
            boolean isFront = (motor.isFlipped() && motor.isStrafeFlipped()) || (!motor.isFlipped() && !motor.isStrafeFlipped());
            motor.get().setTargetPosition(motor.get().getCurrentPosition() + (isFront ? -1 : 1) * motor.getConfiguration().inchesToCounts(inches));
        }

        setUniformDrivePower(power);
        while(op().opModeIsActive() && isAnyDriveTrainMotorBusy()) {
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
     * Strafes the robot for a certain amount of time
     * @param millis The time in milliseconds to strafe
     * @param goLeft If the robot should go left or right
     * @param tolerance How far apart (angle in degrees) it's okay to be off from straight
     */
    public void autoStrafeTimedPID(long millis, boolean goLeft, double tolerance){
        getLogger().log(Level.INFO, "Strafing with a PID, Timed");
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();

        long time = System.currentTimeMillis() + millis;
        double steer, leftSpeed, rightSpeed, target = imu.getYaw();

        while(op().opModeIsActive() && time > System.currentTimeMillis()){
            steer = (goLeft ? 1 : -1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            setIndividualDrivePower(leftSpeed, -leftSpeed, rightSpeed, -rightSpeed);

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
        if(op().opModeIsActive() && Math.abs(angleDiff) > tolerance)
            autoTurnPID(angleDiff, tolerance, true);
    }

    /**
     * Has the robot strafe a set distance using a PID
     * @param inches How far to go in inches. Positive is left, Negative is right
     * @param tolerance How far apart (angle in degrees) it's okay to be off from straight
     */
    public void autoStrafeEncodedPID(double inches, double tolerance){
        getLogger().log(Level.INFO, "Strafing with a PID and Encoders");
        IMU imu = getIMU();
        if(!imu.isRunning()) imu.startIMU();
        resetAllEncoders();
        setAllRunToPosition();

        /* Loops through every motor and sets the position for it to go to. If it's strafe opposite (front right, back left), it subtracts the ticks. Otherwise it adds them. */
        for(DrivetrainMotor motor : getDrivetrainMotors())
            motor.get().setTargetPosition(motor.get().getCurrentPosition() + (motor.isStrafeFlipped() ? -1 : 1) * motor.getConfiguration().inchesToCounts(inches));

        double steer, leftSpeed, rightSpeed, target = imu.getYaw();

        while(op().opModeIsActive() && isAnyDriveTrainMotorBusy()){
            steer = (inches < 0 ? -1 : 1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            setIndividualDrivePower(leftSpeed, -leftSpeed, rightSpeed, -rightSpeed);

            getLogger().putData("Steer", steer);
            if(getDrivetrainMotors().size() > 0) {
                /* TODO make this more clear, not sure WHICH motor has its ticks recorded */
                getLogger().putData("Target Ticks", getDrivetrainMotors().get(0).get().getTargetPosition());
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
        if(op().opModeIsActive() && Math.abs(angleDiff) > tolerance)
            autoTurnPID(angleDiff, tolerance, true);
    }

}
