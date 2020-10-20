package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.NotNull;

import java.util.UUID;

public class IMUHandler extends HardwareComponent {

    private final Robot r;
    private volatile Orientation orientation;
    private volatile Acceleration acceleration;
    private final int updateIntervalMilliseconds = 1000;
    private UUID threadID;

    public IMUHandler(LinearOpMode op, String id, Robot r) {
        super(op, id, HardwareComponentArea.IMU);
        this.r = r;
        try { setComponent(op.hardwareMap.get(BNO055IMU.class, id));
        } catch (Exception e) {
            op.telemetry.addData("Error Adding IMU " + id + ":", e);
            op.telemetry.update();
            op.requestOpModeStop();
            return;
        }

        /* Initialize IMU with desired units and settings */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        get().initialize(parameters);

        updateIMU();
    }

    /** If the robot is running */
    public boolean isRunning(){
        return threadID != null && r.getThread(threadID) != null && r.getThread(threadID).isAlive();
    }

    /** Starts thread to automagically update variables */
    public void startIMU(){
        get().startAccelerationIntegration(new Position(), new Velocity(), updateIntervalMilliseconds);
        threadID = r.addThread(new Thread(() -> {
            while(op.opModeIsActive())
                updateIMU();
        }), true );
    }

    /** Updates important IMU variables */
    public void updateIMU(){
        orientation = get().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        acceleration = get().getGravity();
    }

    @Override
    public BNO055IMU get() {
        return (BNO055IMU) component;
    }

    /**
     * Calculates the distance between two angles
     * @param angle1 A positive number between 0 and 360
     * @param angle2 A positive number between 0 and 360
     * @return A number between 0 and 180
     */
    public static double distanceBetweenAngles(double angle1, double angle2) {
        double angleDistance = Math.abs(angle1 - angle2);
        return angleDistance > 180 ? 360 - angleDistance : angleDistance;
    }

    /** Gets heading (aka 360º version of yaw) of robot in the specified angle unit */
    public float getHeading(@NotNull AngleUnit angleUnit){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.firstAngle));
    }

    /** The -180º to 180º heading */
    public float getYaw(@NotNull AngleUnit angleUnit){
        float h = getHeading(angleUnit);
        return h > -180 && h < 0 ? h + 360 : h;
    }

    /** Gets roll of robot in the specified angle unit */
    public float getRoll(@NotNull AngleUnit angleUnit){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.secondAngle));
    }

    /** Gets pitch of robot in the specified angle unit */
    public float getPitch(@NotNull AngleUnit angleUnit){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.thirdAngle));
    }

}
