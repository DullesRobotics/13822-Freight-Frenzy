package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.NotNull;

import java.util.UUID;
import java.util.logging.Level;

public class IMU extends HardwareComponent {

    private volatile Orientation orientation;
    private volatile Acceleration acceleration;
    private final static int updateIntervalMilliseconds = 1000;
    private UUID threadID;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    /**
     * @param r The robot this IMU is in
     * @param id Usually "IMU"
     */
    public IMU(Robot r, String id) {
        super(r, id, ComponentArea.IMU);
        r.getLogger().log(Level.INFO, "Adding IMU: " + id);
        try { setComponent(r.op().hardwareMap.get(BNO055IMU.class, id));
        } catch (Exception e) {
            r.getLogger().log(Level.SEVERE, "Error Adding IMU " + id, e.toString());
            r.op().requestOpModeStop();
            return;
        }

        /* Initialize IMU with desired units and settings */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        get().initialize(parameters);

        r.getLogger().log(Level.INFO, "Parameterized IMU: " + id);

        updateIMU();
    }

    /** If the robot is running */
    public boolean isRunning(){
        return threadID != null && r.getThread(threadID) != null && r.getThread(threadID).isAlive();
    }

    /** Starts thread to automagically update variables */
    public void startIMU(){
        if(isRunning()) return;
        get().startAccelerationIntegration(new Position(), new Velocity(), updateIntervalMilliseconds);
        threadID = r.addThread(new Thread(() -> {
            while(isRunning() && r.op().opModeIsActive()){
                r.getLogger().putData("IMU Running", isRunning());
                updateIMU();
                r.getLogger().putData("Pitch", String.valueOf(getPitch()));
                r.getLogger().putData("Roll", String.valueOf(getRoll()));
                r.getLogger().putData("Yaw", String.valueOf(getYaw()));
            }
        }), true);
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

    /** Gets heading (aka -180º to 180º version of yaw) of robot in the specified angle unit */
    public float getHeading(){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.firstAngle));
    }

    /** The 360º heading */
    public float getYaw(){
        float h = getHeading();
        return h > -180 && h < 0 ? h + 360 : h;
    }

    /** Gets roll of robot in the specified angle unit */
    public float getRoll(){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.secondAngle));
    }

    /** Gets pitch of robot in the specified angle unit */
    public float getPitch(){
        return angleUnit.normalize(angleUnit.fromUnit(orientation.angleUnit, orientation.thirdAngle));
    }

    /**
     * Sets the angle unit that this imu will return
     * @param angleUnit An angle unit
     */
    public void setAngleUnit(@NotNull AngleUnit angleUnit){
        this.angleUnit = angleUnit;
    }

    /** @return The angle unit of this IMU */
    public AngleUnit getAngleUnit(){
        return angleUnit;
    }
}
