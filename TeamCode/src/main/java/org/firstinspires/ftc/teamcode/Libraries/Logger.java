package org.firstinspires.ftc.teamcode.Libraries;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.Environment;
import android.text.format.DateFormat;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.logging.Level;

@TargetApi(Build.VERSION_CODES.N)
@Deprecated
public class Logger {

    private PrintWriter writer;

    private static final byte linesToShowAtATime = 5;

    private LinearOpMode op;

    private volatile ArrayList<LogEntry> log = new ArrayList<>();

    private volatile HashMap<String, String> dynamicData = new HashMap<>();

    public Logger (LinearOpMode op) {
        this.op = op;
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
        String date = DateFormat.format("MM/dd:HH:mm:SS", Calendar.getInstance().getTime()).toString();
        File logFile = new File(path, formatTextFileName("ftclog_" + date));
        try { writer = new PrintWriter(new FileWriter(logFile, true)); }
        catch (Exception e) {
            e.printStackTrace();
            op.telemetry.addData("ERROR", e);
            op.telemetry.update();
        }
    }

    /**
     * Puts a dynamically logged and updated entry into the log
     * @param dataClassification The key to put in the dynamic logger
     * @param data The data to put in the dynamic logger
     */
    public void putData(String dataClassification, Object data){
        if(!(dynamicData.containsKey(dataClassification) && dynamicData.get(dataClassification).equals(String.valueOf(data)))) {
            dynamicData.put(dataClassification, String.valueOf(data));
            updateFileLog(Level.INFO, dataClassification + ": " + data);
        }
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
    public void log(Level logLevel, String dataClassification, Object data){
        log.add(new LogEntry(logLevel, dataClassification, String.valueOf(data)));
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param data The data itself
     */
    public void log(Level logLevel, String data){
        log.add(new LogEntry(logLevel, null, data));
        updateFileLog(logLevel, data);
    }

      /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param key The data descriptor
     * @param data The data itself
     */
    public void logKeyed(Level logLevel, String key, String data){
        log.add(new LogEntry(logLevel, null, key + ": " + data));
        updateFileLog(logLevel, key + ": " + data);
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

    /** Updates the currently displayed console */
    public void updateConsole() {
//        if(hasChanged) {
//            hasChanged = false;
//            op.telemetry.clearAll();
//            for (int i = 0; i < linesToShowAtATime && i < log.size() - 1; i++) {
//                LogEntry le = log.get(log.size() - i - 1);
//                op.telemetry.addData("[" + le.getLogLevel().getName() + "] " + (le.getDataClassification() == null ? "" : le.getDataClassification()), le.getData());
//            }
//                for (String s : dynamicData.keySet())
//                    op.telemetry.addData(s, dynamicData.get(s));
//            op.telemetry.update();
//        }
}

    /** Updates log file */
    private void updateFileLog(@NotNull Level level, String s){
        String date = DateFormat.format("HH:mm:SS", Calendar.getInstance().getTime()).toString();
        writer.println(date + " [" + level.getName() + "] " + s);
    }

    /** formats a file name */
    @NotNull
    private static String formatTextFileName(String name){
        //TODO replace with single regex
        name = name.replaceAll("/", ""); //forward slash
        name = name.replaceAll("\\\\", ""); // backslash
        name = name.replaceAll("\\x00", ""); //null
        if(!name.endsWith(".txt")) name += ".txt";
        return name;
    }

}
