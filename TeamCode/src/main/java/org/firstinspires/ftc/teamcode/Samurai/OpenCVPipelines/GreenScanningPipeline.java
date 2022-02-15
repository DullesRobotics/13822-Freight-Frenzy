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

public class GreenScanningPipeline extends OpenCvPipeline implements Pipeline {


    public int minThreshold = 155;
    public int maxThreshold = 255;
    private Mat yCrCB;
    private Mat channel;
    private Mat threshold;
    private List<MatOfPoint> contours;

    private Rect rect;
    private Point currentPoint;

    public GreenScanningPipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {

        // for loop that loops through rows
        // for loop that loops through columns
        // get pixel value, store in double[]
        // get green value
        // add up all green values to one variable per zone, and have a incrementing variable per zone

        // take average of all green pixels in each zone

        // get zone with largest average

        return null;
    }



    @Override
    public void updateLog(Logger l) {

    }

}