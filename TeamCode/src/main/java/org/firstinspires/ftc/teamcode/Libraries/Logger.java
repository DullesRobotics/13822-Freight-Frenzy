package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.Level;

public class Logger {

    private static final byte linesToShowAtATime = 5;

    private LinearOpMode op;

    private volatile ArrayList<LogEntry> log = new ArrayList<>();

    private volatile boolean dynamicDataEnabled = false;
    private volatile String dynamicDataHeader = null;
    private volatile HashMap<String, String> dynamicData = new HashMap<>();

    public Logger (LinearOpMode op) {
        this.op = op;
    }

    /**
     * Whether or not to have dynamic data displayed
     * @param b True to display it, false otherwise
     */
    public void usesDynamicData(boolean b){
        dynamicDataEnabled = b;
    }

    /**
     * The dynamic data header to show above the dynamic data
     * @param dynamicDataHeader The data header
     */
    public void setDynamicDataHeader(String dynamicDataHeader){
        this.dynamicDataHeader = dynamicDataHeader;
    }

    /**
     * Puts a dynamically logged and updated entry into the log
     * @param dataClassification The key to put in the dynamic logger
     * @param data The data to put in the dynamic logger
     */
    public void putData(String dataClassification, String data){
        dynamicData.put(dataClassification, data);
    }

    /**
     * Removes a dynamically logged entry
     * @param dataClassification The key of the data to be removed
     */
    public void removeData(String dataClassification){
        dynamicData.remove(dataClassification);
    }

    /** Clears the dynamically logged entries */
    public void clearData(){
        dynamicData.clear();
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param dataClassification The data key
     * @param data The data itself
     */
    public void log(Level logLevel, String dataClassification, String data){
        log.add(new LogEntry(logLevel, dataClassification, data));
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param data The data itself
     */
    public void log(Level logLevel, String data){
        log.add(new LogEntry(logLevel, null, data));
    }

    /** Stores single, non-dynamic log entry */
    private static class LogEntry {

        private Level logLevel;
        private String dataClassification;
        private String data;
        private long date;

        /**
         * @param logLevel Level of importance
         * @param dataClassification The data key
         * @param data The data itself
         */
        public LogEntry (Level logLevel, String dataClassification, String data) {
            this.logLevel = logLevel;
            this.dataClassification = dataClassification;
            this.data = data;
            this.date = System.currentTimeMillis();
        }

        public Level getLogLevel() {
            return logLevel;
        }

        public String getData() {
            return data;
        }

        public String getDataClassification() {
            return dataClassification;
        }

        /** The time this was logged */
        public long getDate() {
            return date;
        }

    }

    public void updateLog() {
        op.telemetry.clearAll();
        for(int i = 0; i < linesToShowAtATime && i < log.size() - 1; i++){
            LogEntry le = log.get(log.size() - i - 1);
            op.telemetry.addData(le.logLevel.getName() + (le.getDataClassification() == null ? "" : le.getDataClassification()), le.getData());
        }
        if(dynamicDataEnabled) {
            if (dynamicDataHeader != null) op.telemetry.addLine(dynamicDataHeader);
            for (String s : dynamicData.keySet())
                op.telemetry.addData(s, dynamicData.get(s));
        }
        op.telemetry.update();
    }

}
