package org.firstinspires.ftc.teamcode.Libraries;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.Environment;
import android.text.format.DateFormat;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.logging.Level;

//@TargetApi(Build.VERSION_CODES.N)
@Deprecated
public class Logger {

    private PrintWriter writer;

    private LinearOpMode op;

    private volatile HashMap<String, Telemetry.Item> items = new HashMap<>();

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
            items.put(dataClassification, op.telemetry.addData(dataClassification, data));
            updateFileLog(Level.INFO, dataClassification + ": " + data);
    }

    /**
     * Removes a dynamically logged entry
     * @param dataClassification The key of the data to be removed
     */
    public void removeData(String dataClassification){
        if(dataClassification != null && items.containsKey(dataClassification))
            op.telemetry.removeItem(items.get(dataClassification));
    }

    /** Clears the dynamically logged entries */
    public void clearData(){
        for(String key : items.keySet())
            op.telemetry.removeItem(items.remove(key));
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param dataClassification The data key
     * @param data The data itself
     */
    public void log(Level logLevel, String dataClassification, Object data){
        String date = DateFormat.format("HH:mm:SS", Calendar.getInstance().getTime()).toString();
        items.put(dataClassification, op.telemetry.addData(date + " [" + logLevel.getName() + "] " + (dataClassification == null ? "" : dataClassification), data.toString()));
        updateFileLog(logLevel, (dataClassification == null ? "" : dataClassification) + ": " + data.toString());
    }

    /**
     * Stores single, non-dynamic log entry
     * @param logLevel Level of importance
     * @param data The data itself
     */
    public void log(Level logLevel, String data){
        String date = DateFormat.format("HH:mm:SS", Calendar.getInstance().getTime()).toString();
        op.telemetry.log().add(date + " [" + logLevel.getName() + "] " + data);
        updateFileLog(logLevel, data);
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
