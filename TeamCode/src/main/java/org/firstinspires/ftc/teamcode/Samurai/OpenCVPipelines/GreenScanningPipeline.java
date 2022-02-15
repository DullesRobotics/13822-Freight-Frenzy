package org.firstinspires.ftc.teamcode.Samurai.OpenCVPipelines;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.Pipeline;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

public class GreenScanningPipeline extends OpenCvPipeline implements Pipeline {


    public int minThreshold = 155;
    public int maxThreshold = 255;
    private Mat yCrCB;
    private Mat channel;
    private Mat threshold;
    private List<MatOfPoint> contours;

    private Rect rect;
    private Point currentPoint;
    private int bestZone = 0;

    public GreenScanningPipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {

        // for loop that loops through rows
        // for loop that loops through columns
        // get pixel value, store in double[]
        // get green value
        // add up all green values to one variable per zone, and have a incrementing variable per zone
        int zone1Colors = 0, zone1Count = 0, zone2Colors=0, zone2Count=0, zone3Colors=0, zone3Count=0;

        for(int row=0;row < input.rows();row++){
            for(int column=0;column < input.cols();column++){
                double[] pixel = input.get(row, column);
                if(row < 107){
                    zone1Count++;
                    zone1Colors+=pixel[1];
                }
                else if(row < 214){
                    zone2Count++;
                    zone2Colors+=pixel[2];
                }
                else{
                    zone3Count++;
                    zone3Colors+=pixel[3];
                }
            }
        }

        // take average of all green pixels in each zone
        int avg1 = zone1Colors/zone1Count;
        int avg2 = zone2Colors/zone2Count;
        int avg3 = zone3Colors/zone3Count;

        // get zone with largest average
        int localZone = Math.max(avg1,Math.max(avg2,avg3));
        bestZone = avg1 == localZone ? 1 : avg2 == localZone ? 2 : 3;

        return null;
    }

    public int getBestZone() {
        return bestZone;
    }

    @Override
    public void updateLog(Logger l) {
        l.log(Level.INFO, "Best Zone: " + bestZone);
    }

}