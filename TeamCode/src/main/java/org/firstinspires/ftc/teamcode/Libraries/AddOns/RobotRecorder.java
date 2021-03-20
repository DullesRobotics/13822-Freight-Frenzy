package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import android.os.SystemClock;

import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.Iterator;
import java.util.TreeMap;
import java.util.UUID;
import java.util.logging.Level;

/**
 * Records controller inputs and plays them back
 */
public class RobotRecorder extends AddOn {

    private Robot r;
    private volatile RecordingState state = RecordingState.NOTHING;
    private volatile Controller masterController;
    private volatile Controller secondaryController;
    private volatile long timeSinceLastResetPress = 0;

    private volatile UUID playBackThreadUUID;
    private volatile UUID recordingThreadUUID;

    /*TODO should I use volatile for these tree maps? I probably should have it cached, though
     * I'm sacrificing it being more thread-safe */
    private TreeMap<Long, byte[]> actions1 = new TreeMap<>();
    private TreeMap<Long, byte[]> actions2 = new TreeMap<>();
    private volatile long lastUpdated1 = 0, lastUpdated2 = 0;

    /**
     * Initializes and possibly starts this robot recorder.
     */
    public RobotRecorder () {
        super(AddOnType.ROBOT_RECORDER);
    }

    @Override
    protected void initAO(Robot r) {
        this.r = r;
        this.masterController = r.ctrl1();
        this.secondaryController = r.ctrl2();
    }

    /**
     * Collects button presses to alter the robot recorder state. </br>
     * Does NOT record button presses
     */
    @Override
    protected void startAO(){
        r.addThread(new Thread(() -> {
            while(r.op().opModeIsActive()) {
                if(masterController == null || secondaryController == null) return;
                if (masterController.asStandard().buttonUp()) startRecording();
                if (masterController.asStandard().buttonLeft()) beginPlayback();
                if (masterController.asStandard().buttonDown()) stopAO();
                /*you must double press to reset*/
                if (masterController.asStandard().buttonRight())
                    if (System.currentTimeMillis() - timeSinceLastResetPress > 800)
                        timeSinceLastResetPress = System.currentTimeMillis();
                    else
                        resetRecording();
            }
        }), true);
    }

    /**
     * Records controller data. <br>
     * Whenever a Gamepad updates (e.g. button pushed, joystick nudged), its timestamp does too. <br>
     * We check if the timestamp has updated from its previous, and if so we add a
     * new controller state to the map. <br>
     * A controller state is stored in a byte map that can be later decoded in a specially
     * made method in the {@link org.firstinspires.ftc.teamcode.Hardware.Controller} class.
     */
    private void startRecording(){
        if(changeState(RecordingState.RECORDING)) {
            recordingThreadUUID = r.addThread(new Thread(() -> {
                long startTime = SystemClock.uptimeMillis();
                try {
                    while(r.op().opModeIsActive()) {
                        if (r.op().gamepad1.timestamp > lastUpdated1) {
                            lastUpdated1 = r.op().gamepad1.timestamp - startTime;
                            actions1.put(lastUpdated1, r.op().gamepad1.toByteArray());
                        }
                        if (r.op().gamepad2.timestamp > lastUpdated2) {
                            lastUpdated2 = r.op().gamepad2.timestamp - startTime;
                            actions2.put(lastUpdated2, r.op().gamepad2.toByteArray());
                        }
                    }
                } catch (RobotCoreException e) {
                    r.getLogger().log(Level.SEVERE, "There was an error collecting " +
                            "controller data. Recording has stopped and all data has been reset.");
                    resetRecording(); //ALWAYS run last in a thread
                }
            }), true);
        }
    }

    /** Resets the stored recording data. */
    private void resetRecording(){
        stopAO();
        actions1.clear();
        actions2.clear();
    }

    /**
     * Plays back recorded controller maps <br>
     * This iterates through each action for both controllers, and updates the specific middle-man
     * Controller object accordingly. It does this at the right time as well.
     */
    @SuppressWarnings("uncheckedcast")
    private void beginPlayback(){
        if(actions1.isEmpty() && actions2.isEmpty()) return;
        r.setAutoMode(true);
        if(changeState(RecordingState.PLAYBACK)) {
            playBackThreadUUID = r.addThread(new Thread(() -> {
                long startTime = SystemClock.uptimeMillis();
                try {
                    Iterator<Long> timeStampIterator1 = actions1.keySet().iterator(),
                            timeStampIterator2 = actions1.keySet().iterator();
                    long nextTimeStamp1 = timeStampIterator1.next(),
                            nextTimeStamp2 = timeStampIterator2.next();
                    while(r.op().opModeIsActive()){
                        if(nextTimeStamp1 != -999 && SystemClock.uptimeMillis() - startTime > nextTimeStamp1){
                            byte[] controllerMap = actions1.get(nextTimeStamp1);
                            nextTimeStamp1 = timeStampIterator1.hasNext() ? timeStampIterator1.next() : -999;
                            if(controllerMap != null) masterController.setAutoModeValues(controllerMap);
                        }
                        if(nextTimeStamp2 != -999 && SystemClock.uptimeMillis() - startTime > nextTimeStamp2){
                            byte[] controllerMap = actions2.get(nextTimeStamp2);
                            nextTimeStamp2 = timeStampIterator2.hasNext() ? timeStampIterator2.next() : -999;
                            if(controllerMap != null) secondaryController.setAutoModeValues(controllerMap);
                        }
                    }
                } catch (RobotCoreException e) {
                    r.getLogger().log(Level.SEVERE, "There was an error playing back " +
                            "controller data. Playback has stopped.");
                    stopAO();
                }
            }), true, () -> r.setAutoMode(false));
        }
    }

    /** Stops both the recorder and playback. */
    @Override
    protected void stopAO(){
        changeState(RecordingState.NOTHING);
    }

    /**
     * stops threads that would interfere with newly running ones
     * @param state The state to change the Recorder to
     * @return If the new state is not identical to the current one
     */
    private boolean changeState(RecordingState state){
        if(this.state == state) return false;
        /* Checks the CURRENT state to determine what to stop */
        lastUpdated1 = 0;
        lastUpdated2 = 0;
        switch(this.state){
            case RECORDING: if(recordingThreadUUID != null && r.getThread(recordingThreadUUID) != null) r.getThread(recordingThreadUUID).interrupt(); break;
            case PLAYBACK: if(playBackThreadUUID != null && r.getThread(playBackThreadUUID) != null) r.getThread(playBackThreadUUID).interrupt(); break;
        }
        this.state = state;
        return true;
    }

    private enum RecordingState {
        RECORDING,
        PLAYBACK,
        NOTHING
    }

}
