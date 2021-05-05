package org.firstinspires.ftc.teamcode.Libraries;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.Environment;
import android.text.format.DateFormat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.ConcurrentModificationException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.logging.Level;

//@TargetApi(Build.VERSION_CODES.N)
public class Logger {

    private final static long DATA_UPDATE_MIN_MILLIS = 100;
    private long lastDashUpdateTime = System.currentTimeMillis();

    private PrintWriter writer;

    private LinearOpMode op;

    private volatile HashMap<String, Object> items = new HashMap<>();
    private volatile HashMap<String, Telemetry.Item> telemItems = new HashMap<>();
    private volatile HashMap<String, Long> itemUpdate = new HashMap<>();

    public Logger (LinearOpMode op) {
        this.op = op;
        File path = Environment.getExternalStoragePublicDirectory("FIRST/logs");
        String date = DateFormat.format("MM/dd:HH:mm:ss", Calendar.getInstance().getTime()).toString();
        File logFile = new File(path, formatTextFileName("ftclog_" + date));
        try { writer = new PrintWriter(new FileWriter(logFile, true)); }
        catch (Exception e) {
            e.printStackTrace();
            //op.telemetry.addData("ERROR", e);
            //op.telemetry.update();
        }
    }

    /**
     * Puts a dynamically logged and updated entry into the log
     * @param dataClassification The key to put in the dynamic logger
     * @param data The data to put in the dynamic logger
     */
    public void putData(String dataClassification, Object data){
        if(!items.containsKey(dataClassification) || (!items.get(dataClassification).equals(data) && itemUpdate.get(dataClassification) + DATA_UPDATE_MIN_MILLIS < System.currentTimeMillis())) {
            items.put(dataClassification, data);
            telemItems.put(dataClassification, op.telemetry.addData(dataClassification, data));
            itemUpdate.put(dataClassification, System.currentTimeMillis());
            updateFileLog(Level.INFO, dataClassification + ": " + data);
        }
    }

    /**
     * Removes a dynamically logged entry
     * @param dataClassification The key of the data to be removed
     */
    public void removeData(String dataClassification){
        if(dataClassification != null){
            op.telemetry.removeItem(telemItems.get(dataClassification));
            items.remove(dataClassification);
            telemItems.remove(dataClassification);
        }
    }

    /** Clears the dynamically logged entries */
    public void clearData(){
        for(Telemetry.Item item : telemItems.values())
            op.telemetry.removeItem(item);
        items.clear();
        telemItems.clear();
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param dataClassification The data key
     * @param data The data itself
     */
    public void log(Level logLevel, String dataClassification, Object data){
        String date = DateFormat.format("HH:mm:ss", Calendar.getInstance().getTime()).toString();
        String text = date + " [" + logLevel.getName() + "] " + (dataClassification == null ? "" : dataClassification + ": ") + data;
        op.telemetry.log().add(text);
        updateFileLog(logLevel, (dataClassification == null ? "" : dataClassification + ": ") + data);
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param data The data itself
     */
    public void log(Level logLevel, String data){
        String date = DateFormat.format("HH:mm:ss", Calendar.getInstance().getTime()).toString();
        String text = date + " [" + logLevel.getName() + "] " + data;
        op.telemetry.log().add(text);
        updateFileLog(logLevel, data);
    }

    public void updateLog(){
        boolean updateDash = lastDashUpdateTime + DATA_UPDATE_MIN_MILLIS < System.currentTimeMillis();
        TelemetryPacket packet = new TelemetryPacket();
        try {
            for(String s : items.keySet()) {
                Object data = items.get(s);
                op.telemetry.addData(s, data == null ? "null" : data);
                if(updateDash) {
                 //   packet.addLine(s + ": " + (data == null ? "null" : data));
                    FtcDashboard.getInstance().getTelemetry().addData(s, data);
                }
            }
            if(updateDash) {
                // FtcDashboard.getInstance().sendTelemetryPacket(packet);
                FtcDashboard.getInstance().getTelemetry().update();
            }
        } catch (ConcurrentModificationException ignored) {}
//        op.telemetry.update();
    }

    /** Updates log file */
    private void updateFileLog(@NotNull Level level, String s){
        String date = DateFormat.format("HH:mm:ss", Calendar.getInstance().getTime()).toString();
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
