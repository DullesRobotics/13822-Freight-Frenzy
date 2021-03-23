package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.PID;

import java.util.logging.Level;

//@TargetApi(Build.VERSION_CODES.N)
public class StandardDriveTrain extends DriveTrain {

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public StandardDriveTrain(LinearOpMode op, PID pid) {
        super(op, pid);
    }

    /**
     * Takes in super initiators
     * @param op The op mode this is used for
     */
    public StandardDriveTrain(LinearOpMode op) {
        super(op);
    }

    /**
     * Drives using the joystick with the defined speed <br/>
     * By pressing the right or left trigger, you can enter precision mode
     * @param ctrl The controller to move the robot with
     */
    public void driveWithController(Controller ctrl){
        getLogger().log(Level.INFO, "Beginning drive with controller, standard");
        addThread(new Thread(() -> {
            double currentSpeed;
            while(op().opModeIsActive()){
                /* linear equation to calculate speed based on right trigger's position */
                currentSpeed = ctrl.rightTrigger() > 0 || ctrl.leftTrigger() > 0 ? precisionSpeed : speed;
                getLogger().putData("Motor Speed", currentSpeed);
                setSidedDrivePower(-1 * currentSpeed * ctrl.leftY(), -1 * currentSpeed * ctrl.rightY());
            }
        }), true);
    }

    /**
     * Moves the robot for a certain amount of time
     * @param millis The amount of milliseconds to move the robot
     * @param direction Direction to go
     */
    public void autoDriveTimed(long millis, Direction direction){
        if(direction == null) direction = Direction.FORWARD;

        getLogger().log(Level.INFO, "Moving ( direction = " + direction.toString() + " ) for " + millis + " milliseconds.");
        long time = System.currentTimeMillis() + millis;
        while(op().opModeIsActive() && time >= System.currentTimeMillis()) {
            getLogger().putData("Time Left", time - System.currentTimeMillis());

            double leftPower = speed, rightPower = speed;
            switch(direction) {
                case LEFT: leftPower = -speed; rightPower = speed; break;
                case RIGHT: leftPower = speed; rightPower = -speed; break;
            }

            setSidedDrivePower(leftPower, rightPower);
        }
        setUniformDrivePower(0);
        getLogger().removeData("Time Left");
        getLogger().log(Level.INFO, "Done moving.");
    }

//    /**
//     * Moves robot forward using a set time and a PID
//     * @param millis The amount of milliseconds to move the robot
//     * @param tolerance what angle the robot can have moved to be okay
//     */
//    public void autoDriveForwardTimedPID(long millis, double tolerance, boolean forward){
//        if(pid != null){
//            getLogger().log(Level.WARNING, "PID Null");
//            return;
//        }
//        getLogger().log(Level.INFO, "Driving Forward with PID and Timed");
//        long time = System.currentTimeMillis() + millis;
//        IMU imu = getIMU();
//        if(!imu.isRunning()) imu.startIMU();
//        resetAllEncoders();
//
//        double steer, leftSpeed, rightSpeed, target = imu.getYaw();
//        while(op().opModeIsActive() && time >= System.currentTimeMillis()){
//            steer = (forward ? 1 : -1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
//            leftSpeed = speed - steer;
//            rightSpeed = speed + steer;
//            setSidedDrivePower(leftSpeed, rightSpeed);
//            getLogger().putData("Steer", steer);
//            getLogger().putData("Time Left", time - System.currentTimeMillis());
//            getLogger().putData("Speed (L, R)", "(" + leftSpeed + ", " + rightSpeed + ")");
//        }
//        getLogger().log(Level.INFO, "Angle to turn: ", target - imu.getYaw());
//        op().sleep(400L);
//        getLogger().clearData();
//        setUniformDrivePower(0);
//        getLogger().log(Level.INFO, "Finished Driving Forward");
//
//        /* Correcting angle to make sure it stays in the same line */
//        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
//        if(op().opModeIsActive() && Math.abs(angleDiff) > 1)
//            autoTurnPID(angleDiff, tolerance, true);
//    }
//
//    /**
//     * Moves robot forward using encoders and a PID
//     * @param inches the distance to move forward in inches
//     * @param tolerance how far in degress the pid can be off
//     * @param tolerance what angle the robot can have moved to be okay
//     */
//    public void autoStraightEncodedPID(double inches, double tolerance){
//        if(pid != null){
//            getLogger().log(Level.WARNING, "PID Null");
//            return;
//        }
//        getLogger().log(Level.INFO, "Driving Forward with PID and Encoded");
//        IMU imu = getIMU();
//        if(!imu.isRunning()) imu.startIMU();
//        resetAllEncoders();
//        setAllRunToPosition();
//
//        for(DrivetrainMotor m : getDrivetrainMotors())
//            m.get().setTargetPosition(m.get().getCurrentPosition() + m.getConfiguration().inchesToCounts(inches));
//
//        double steer, leftSpeed, rightSpeed, target = imu.getYaw();
//        while(op().opModeIsActive() && isAnyDriveTrainMotorBusy()){
//            steer = (inches < 0 ? -1 : 1) * pid.update(PID.PIDType.THREE_SIXTY_ANGLE, imu.getYaw(), target);
//            leftSpeed = speed - steer;
//            rightSpeed = speed + steer;
//            setSidedDrivePower(leftSpeed, rightSpeed);
//
//            getLogger().putData("Steer", steer);
//            if(getDrivetrainMotors().size() > 0) {
//                getLogger().putData("Target Ticks", getDrivetrainMotors().get(0).get().getTargetPosition());
//                getLogger().putData("Current Ticks", steer);
//            }
//            getLogger().putData("Speed (L, R)", "(" + leftSpeed + ", " + rightSpeed + ")");
//        }
//        getLogger().log(Level.INFO, "Angle to turn", target - imu.getYaw());
//        op().sleep(400L);
//        setUniformDrivePower(0);
//        setAllRunWithEncoder();
//        getLogger().clearData();
//        getLogger().log(Level.INFO, "Finished Driving Forward");
//
//        /* Correcting angle to make sure it stays in the same line */
//        double angleDiff = IMU.distanceBetweenAngles(imu.getYaw(), target);
//        if(op().opModeIsActive() && Math.abs(angleDiff) > tolerance)
//            autoTurnPID(angleDiff, tolerance, true);
//    }
//
//
//    /**
//     * Turns robot a certain angle with the PID
//     * @param angle The angle to move the robot
//     * @param tolerance The angle where the robot doesn't have to correct further
//     * @param correction If this is a small correction, uses a smaller speed
//     */
//    public void autoTurnPID(double angle, double tolerance, boolean correction) {
//        if(pid != null){
//            getLogger().log(Level.WARNING, "PID Null");
//            return;
//        }
//        getLogger().log(Level.INFO, "Turning with PID, Correction" + correction);
//        double tempSpeed = correction ? 0.2 : speed;
//        IMU imu = getIMU();
//        if(!imu.isRunning()) imu.startIMU();
//        double startYaw = imu.getYaw(), targetYaw = imu.getYaw() + angle;
//
//        /* Correcting the angle just in case */
//        targetYaw = targetYaw > 360 ? targetYaw % 360 : targetYaw;
//        targetYaw = targetYaw < 0 ? targetYaw + 360 : targetYaw;
//
//        double distanceToTarget = IMU.distanceBetweenAngles(startYaw, targetYaw);
//
//        for (int i = 1; i <= 8; i++) {
//            while (op().opModeIsActive() && (distanceToTarget < tolerance) ^ (distanceToTarget > -tolerance)) {
//                distanceToTarget = IMU.distanceBetweenAngles(startYaw, targetYaw);
//                if (distanceToTarget < 0 + tolerance)
//                    if (angle > 0)
//                        setTurningDrivePower(tempSpeed / i);
//                    else
//                        setTurningDrivePower(tempSpeed / i);
//                else if (distanceToTarget > 0 - tolerance)
//                    if (angle > 0)
//                        setTurningDrivePower(tempSpeed / i);
//                    else
//                        setTurningDrivePower(-tempSpeed / i);
//                else {
//                    setUniformDrivePower(0);
//                    break;
//                }
//                getLogger().putData("Distance to target", distanceToTarget);
//                getLogger().putData("Target Yaw", targetYaw);
//                getLogger().putData("Start Yaw", startYaw);
//                getLogger().putData("Angle", angle);
//                getLogger().putData("Iteration", i);
//            }
//            op().sleep(200);
//        }
//        getLogger().clearData();
//        getLogger().log(Level.INFO, "PID Done Turning");
//        setUniformDrivePower(0);
//    }
//
//    /**
//     * Moves the robot forward a specified number of inches
//     * @param inches The number of inches to move
//     * @param direction what direction to turn
//     */
//    public void autoDriveEncoded(double inches, Direction direction){
//        setAllRunToPosition();
//
//        double leftPower = speed, rightPower = speed;
//        switch(direction) {
//            case LEFT: leftPower = -speed; rightPower = speed; break;
//            case RIGHT: leftPower = speed; rightPower = -speed; break;
//        }
//
//        for(DrivetrainMotor motor : getDrivetrainMotors()) {
//            int currentPosition = motor.get().getCurrentPosition();
//            int ticksToAdd = motor.getConfiguration().inchesToCounts(inches);
//            motor.get().setTargetPosition(currentPosition + ticksToAdd);
//        }
//
////        for(Motor motor : getDriveTrainMotors())
////            motor.get().setTargetPosition(motor.get().getCurrentPosition() +
////                    ((motor.isOpposite() && turnRight) || (!motor.isOpposite() && !turnRight) || !turn ? 1 : -1 ) *
////                            (motor.getConfiguration().inchesToCounts(inches)));
//
////        double power = inches < 0 ? -speed : speed;
////        setUniformDrivePower(power);
//
//       setSidedDrivePower(leftPower, rightPower);
//
//        while(op().opModeIsActive() && isAnyDriveTrainMotorBusy())
//            setSidedDrivePower(leftPower, rightPower);
//
//        setUniformDrivePower(0);
//    }

}
