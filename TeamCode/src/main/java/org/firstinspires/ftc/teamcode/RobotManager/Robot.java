package org.firstinspires.ftc.teamcode.RobotManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.HardwareComponent;
import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.DrivetrainMotor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.MotorType;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.Libraries.AddOns.AddOnHandler;
import org.firstinspires.ftc.teamcode.Libraries.IMU;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.UUID;
import java.util.logging.Level;

/**
 * Hello, code begins here :D
 */
//@TargetApi(Build.VERSION_CODES.N)
public class Robot {

    private volatile LinearOpMode op;
    private volatile Controller controller1, controller2;
    private volatile ArrayList<HardwareComponent> hardwareComponents = new ArrayList<>();

    private volatile HashMap<UUID, Thread> runningThreads = new HashMap<>();
    private volatile HashMap<UUID, Runnable> endingRunnables = new HashMap<>();

    private volatile AddOnHandler addOnHandler;
    private volatile Logger logger;
    private volatile HashMap<String, Object> state = new HashMap<>();

    protected Robot(LinearOpMode op){
        this.op = op;
        logger = new Logger(op);
        getLogger().log(Level.INFO, "---CONSOLE---");
        controller1 = new Controller(op.gamepad1);
        controller2 = new Controller(op.gamepad2);
        addOnHandler = new AddOnHandler(this);
    }

    /** Add hardware to the robot array
     * @param hardware A list of hardware to add to the array */
    public void addHardware(HardwareComponent... hardware){
        this.hardwareComponents.addAll(Arrays.asList(hardware));
        for(HardwareComponent hardwareComponent : hardware)
            getLogger().log(Level.INFO, "Added Hardware Component", hardwareComponent.toString());
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
     * Returns the drive train motor with the matching position
     * @param drivetrainPosition position of component on the drivetrain
     * @return the hardware component
     */
    @Nullable
    public DrivetrainMotor getDrivetrainMotor(MotorType.DrivetrainPosition drivetrainPosition){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof DrivetrainMotor && ((DrivetrainMotor) hdw).getDrivetrainPosition() == drivetrainPosition)
                return (DrivetrainMotor) hdw;
        return null;
    }

    /**
     * Returns the drive train motors in the specified hardware area
     * @return the hardware components
     */
    public ArrayList<DrivetrainMotor> getDrivetrainMotors(){
        ArrayList<DrivetrainMotor> ar = new ArrayList<>();
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof DrivetrainMotor)
                ar.add((DrivetrainMotor) hdw);
        return ar;
    }

    /**
     * Returns the motor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public Motor getMotor(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof Motor && hdw.getId().equals(id))
                return (Motor) hdw;
        return null;
    }

    /**
     * Returns the motors in the specified hardware area
     * @param area the area where the hardware components are
     * @return the hardware components
     */
    public ArrayList<Motor> getMotors(ComponentArea area){
        ArrayList<Motor> ar = new ArrayList<>();
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof Motor && hdw.getComponentArea() == area)
                ar.add((Motor) hdw);
        return ar;
    }

    /**
     * Returns the servo with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public Servo getServo(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof Servo && hdw.getId().equals(id))
                return (Servo) hdw;
        return null;
    }

    /**
     * Returns the servos in the specified hardware area
     * @param area the area where the hardware components are
     * @return the hardware components
     */
    public ArrayList<Servo> getServos(ComponentArea area){
        ArrayList<Servo> ar = new ArrayList<>();
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof Servo && hdw.getComponentArea() == area)
                ar.add((Servo) hdw);
        return ar;
    }

    /**
     * Returns the touch sensor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public TouchSensor getTouchSensor(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof TouchSensor && hdw.getId().equals(id))
                return (TouchSensor) hdw;
        return null;
    }

    /**
     * Returns the color sensor with the matching ID
     * @param id id of hardware component
     * @return the hardware component
     */
    @Nullable
    public ColorSensor getColorSensor(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof ColorSensor && hdw.getId().equals(id))
                return (ColorSensor) hdw;
        return null;
    }

    /**
     * Gets an IMU with the matching ID
     * @param id id of the IMU
     * @return the hardware component
     */
    @Nullable
    public IMU getIMU(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof IMU && hdw.getId().equals(id))
                return (IMU) hdw;
        return null;
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
            for(HardwareComponent hdw : hardwareComponents) {
                imu = hdw instanceof IMU ? (IMU) hdw : null;
                break;
            }

        return imu;
    }

    /**
     * Gets a USB Webcam with the matching ID
     * @param id id of the USB Webcam
     * @return the hardware component
     */
    @Nullable
    public USBWebcam getUSBWebcam(String id){
        for(HardwareComponent hdw : hardwareComponents)
            if(hdw instanceof USBWebcam && hdw.getId().equals(id))
                return (USBWebcam) hdw;
        return null;
    }

    /**
     * Gets an IMU with the default ID "IMU"
     * If there is no IMU with the default ID, it gets the first IMU in the ArrayList
     * @return the hardware component
     */
    @Nullable
    public USBWebcam getUSBWebcam(){
        USBWebcam webcam = getUSBWebcam("Webcam");
        if(webcam == null)
            for(HardwareComponent hdw : hardwareComponents) {
                webcam = hdw instanceof USBWebcam ? (USBWebcam) hdw : null;
                break;
            }
        return webcam;
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
        for(Thread t : runningThreads.values())
            try { t.interrupt(); }
            catch (Exception ignored) {}
        runningThreads.clear();
        if(op.opModeIsActive())
            for(Runnable r : endingRunnables.values())
                r.run();
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

    /**
     * Set if the controller should be locked
     * @param autoMode What the controller lock state should be
     */
    public void setAutoMode(boolean autoMode) {
        controller1.setAutoMode(autoMode);
        controller2.setAutoMode(autoMode);
    }

    /**
     * Gets the central add-on handler.
     * @return The Add-On Handler for this robot.
     */
    public AddOnHandler addOnManager(){
        return addOnHandler;
    }

    /** @return The op mode for this robot */
    public LinearOpMode op(){
        return op;
    }

    /**
     * Gets this robot's data in storage. <br>
     * Data only exists for the duration of the op mode's use,
     * so it's only stored in memory.
     * @return The state HashMap of the robot
     */
    public HashMap<String, Object> getState() {
        return state;
    }


}
