package org.firstinspires.ftc.teamcode.RobotManager;

import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.firstinspires.ftc.teamcode.Libraries.RobotRecorder;
import org.firstinspires.ftc.teamcode.OpModes.HardwareConfigurator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.UUID;
import java.util.logging.Level;
import java.util.stream.Collectors;

/**
 * Hello, code begins here :D
 */
@TargetApi(Build.VERSION_CODES.N)
public class Robot {

    public volatile LinearOpMode op;
    private volatile Controller controller1, controller2;
    private volatile ArrayList<HardwareComponent> hardwareComponents = new ArrayList<>();

    private volatile HashMap<UUID, Thread> runningThreads = new HashMap<>();
    private volatile HashMap<UUID, Runnable> endingRunnables = new HashMap<>();

    private volatile Logger logger;
    private volatile RobotRecorder recorder;

    protected Robot(LinearOpMode op, boolean hasRecorder){
        this.op = op;
        controller1 = new Controller(op.gamepad1);
        controller2 = new Controller(op.gamepad2);
        logger = new Logger(op);
        if(hasRecorder) recorder = new RobotRecorder(this, ctrl1(), ctrl2(), true);
        startLogger();
        HardwareConfigurator.configureHardware(this);
    }

    /** Add hardware to the robot array
     * @param hardware A list of hardware to add to the array */
    public void addHardware(HardwareComponent... hardware){
        this.hardwareComponents.addAll(Arrays.asList(hardware));
        Arrays.stream(hardware).forEach(hardwareComponent -> getLogger().log(Level.INFO, "Added Hardware Component", hardwareComponent.toString()));
    }

    /** @return Controller 1 */
    public Controller ctrl1() {
        return controller1;
    }
    /** @return Controller2 */
    public Controller ctrl2() {
        return controller2;
    }

    /**
     * Returns the motor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public Motor getMotor(String id){
        return (Motor) hardwareComponents.stream().filter(hdw -> hdw instanceof Motor && hdw.getId().equals(id)).findFirst().orElse(null);
    }

    /**
     * Returns the motors in the specified hardware area
     * @param area the area where the hardware components are
     * @return the hardware components
     */
    public ArrayList<Motor> getMotors(HardwareComponentArea area){
        return hardwareComponents.stream().filter(hdw -> hdw instanceof Motor && hdw.getComponentArea() == area).map(hdw -> (Motor) hdw).collect(Collectors.toCollection(ArrayList::new));
    }

    /**
     * Returns the servo with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public Servo getServo(String id){
        return (Servo) hardwareComponents.stream().filter(hdw -> hdw instanceof Servo && hdw.getId().equals(id)).findFirst().orElse(null);
    }

    /**
     * Returns the servos in the specified hardware area
     * @param area the area where the hardware components are
     * @return the hardware components
     */
    public ArrayList<Servo> getServos(HardwareComponentArea area){
        return hardwareComponents.stream().filter(hdw -> hdw instanceof Servo && hdw.getComponentArea() == area).map(hdw -> (Servo) hdw).collect(Collectors.toCollection(ArrayList::new));
    }

    /**
     * Returns the touch sensor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public TouchSensor getTouchSensor(String id){
        return (TouchSensor) hardwareComponents.stream().filter(hdw -> hdw instanceof TouchSensor && hdw.getId().equals(id)).findFirst().orElse(null);
    }

    /**
     * Returns the color sensor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public ColorSensor getColorSensor(String id){
        return (ColorSensor) hardwareComponents.stream().filter(hdw -> hdw instanceof ColorSensor && hdw.getId().equals(id)).findFirst().orElse(null);
    }

    /**
     * Gets an IMU with the matching ID
     * @param id id of the IMU
     * @return the hardware component
     */
    @Nullable
    public IMU getIMU(String id){
        return (IMU) hardwareComponents.stream().filter(hdw -> hdw instanceof IMU && hdw.getId().equals(id)).findFirst().orElse(null);
    }

    /**
     * Gets an IMU with the default ID "IMU"
     * If there is no IMU with the default ID, it gets the first IMU in the ArrayList
     * @return the hardware component
     */
    @Nullable
    public IMU getIMU(){
        IMU imu = getIMU("IMU");
        if(imu == null)
            imu = (IMU) hardwareComponents.stream().filter(hdw -> hdw instanceof IMU).findFirst().orElse(null);
        return imu;
    }

    /**
     * Adds a running thread to remember
     * @param t The thread to remember
     * @param autoStart Starts the thread passed in
     * @return The ID of the thread for reference
     */
    public UUID addThread(Thread t, boolean autoStart){
        UUID uuid = UUID.randomUUID();
        runningThreads.put(uuid, t);
        if(autoStart) t.start();
        return uuid;
    }

    /**
     * Adds a running thread to remember
     * @param t The thread to remember
     * @param autoStart Starts the thread passed in
     * @param endRunnable A runnable to run when the thread is stopped. The ending runnable should not add threads with ending runnables.
     * @return The ID of the thread for reference
     */
    public UUID addThread(Thread t, boolean autoStart, Runnable endRunnable){
        UUID uuid = UUID.randomUUID();
        runningThreads.put(uuid, t);
        endingRunnables.put(uuid, endRunnable);
        if(autoStart) t.start();
        return uuid;
    }

    /** Stops all running threads */
    public void stopAllThreads() {
        runningThreads.forEach((s, thread) -> {
            try { thread.interrupt(); }
            catch (Exception ignored) {}
        });
        runningThreads.clear();
        if(op.opModeIsActive())
            endingRunnables.forEach((uuid, runnable) -> runnable.run());
        endingRunnables.clear();
    }

    /**
     * Returns the thread with the specified ID
     * @param uuid The ID of the thread
     * @return The thread with the id
     */
    @Nullable
    public Thread getThread(@NotNull UUID uuid){
        return runningThreads.get(uuid);
    }

    public Logger getLogger(){ return logger; }

    /** Starts the logger thread */
    private void startLogger(){
        addThread(new Thread(()->{
            while(op.opModeIsActive())
                logger.updateConsole();
        }), true);
    }

    /**
     * Set if the controller should be locked
     * @param autoMode What the controller lock state should be
     */
    public void setAutoMode(boolean autoMode) {
        controller1.setAutoMode(autoMode);
        controller2.setAutoMode(autoMode);
    }
}
