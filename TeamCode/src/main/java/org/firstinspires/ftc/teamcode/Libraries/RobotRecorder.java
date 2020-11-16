package org.firstinspires.ftc.teamcode.Libraries;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.Objects;
import java.util.TreeMap;
import java.util.UUID;

/**
 * Records controller inputs and plays them back
 */
public class RobotRecorder {

    private volatile RecordingState state = RecordingState.NOTHING;
    private Robot r;
    private Controller c;
    private TreeMap<Long, byte[]> actions = new TreeMap<>();
    private long timeSinceLastResetPress = 0;
    private UUID playBackThreadUUID;
    private UUID recordingThreadUUID;

    /**
     * Initializes and possibly starts this robot recorder.
     * @param r The robot to record
     * @param c The controller to listen to for control inputs
     * @param start If the robot recorder should start on initialization
     */
    public RobotRecorder (Robot r, Controller c, boolean start) {
        this.r = r;
        this.c = c;
        if(start) collectStates();
    }

    /**
     * Collects button presses to alter the robot recorder state.
     */
    public void collectStates(){
        r.addThread(new Thread(() -> {
            if(c.buttonUp()) startRecording();
            if(c.buttonLeft()) beginPlayback();
            if(c.buttonDown()) stop();
            /*you must double press to reset*/
            if(c.buttonRight())
                if(System.currentTimeMillis() - timeSinceLastResetPress > 800)
                    timeSinceLastResetPress = System.currentTimeMillis();
                else
                    resetRecording();
        }), true);
    }

    /**
     *
     */
    public void startRecording(){
        if(changeState(RecordingState.RECORDING)) {
            recordingThreadUUID = r.addThread(new Thread(() -> {

            }), true);
        }
    }

    /**
     * Resets the actions stored.
     */
    public void resetRecording(){
        stop();
        actions.clear();
    }

    /**
     *
     */
    public void beginPlayback(){
        if(changeState(RecordingState.PLAYBACK)) {
            playBackThreadUUID = r.addThread(new Thread(() -> {

            }), true);
        }
    }

    /**
     *
     */
    public void stop(){
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
            switch(this.state){
            case RECORDING: if(recordingThreadUUID != null && r.getThread(recordingThreadUUID) != null) r.getThread(recordingThreadUUID).interrupt(); break;
            case PLAYBACK: if(playBackThreadUUID != null && r.getThread(playBackThreadUUID) != null) r.getThread(playBackThreadUUID).interrupt(); break;
        }
        this.state = state;
        return true;
    }

    enum RecordingState {
        RECORDING,
        PLAYBACK,
        NOTHING
    }

}
