package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashMap;
import java.util.TreeMap;

public class RobotRecorder {

    private volatile RecordingState state = RecordingState.NOTHING;
    private LinearOpMode op;
    private TreeMap<Long, byte[]> actions = new TreeMap<>();

    public RobotRecorder (LinearOpMode op) {
        this.op = op;
    }

    public void startRecording(){

    }

    public void resetRecording(){

    }

    public void beginPlayback(){

    }

    public void stop(){

    }

    enum RecordingState {
        RECORDING,
        PLAYBACK,
        NOTHING
    }

}
