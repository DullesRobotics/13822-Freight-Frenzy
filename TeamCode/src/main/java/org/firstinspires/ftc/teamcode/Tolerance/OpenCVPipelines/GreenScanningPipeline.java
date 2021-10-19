package org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.Pipeline;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GreenScanningPipeline extends OpenCvPipeline implements Pipeline {

    /* color constants */
    private static final Scalar BLUE = new Scalar(0, 0, 255), GREEN = new Scalar(0, 255, 0);

    public static short Y_MAX = 30, Y_MIN = 200;
    public static short LEFT_2_MID_X = 107, MID_2_RIGHT_X = 214;

    private volatile Region selectedZone = Region.RIGHT;
    private volatile double leftWeight = 0, rightWeight = 0, middleWeight = 0;

    /*
     * Working variables
     */
    private Mat region1_Cb, region2_Cb, region3_Cb, YCrCb = new Mat(), Cb = new Mat();
    private int avg1;

    public enum Region {
        LEFT, MIDDLE, RIGHT
    }

    @Override
    public void updateLog(Logger l) {
        l.putData("Selected Zone", selectedZone);
        l.putData("Weights (L, M, R)", leftWeight + ", " + middleWeight + ", " + rightWeight);
    }

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(new Point(0, Y_MAX), new Point(320, Y_MIN)));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                new Point(0, Y_MAX), // First point which defines the rectangle
                new Point(320, Y_MIN), // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        selectedZone = Region.MIDDLE;
//        if(avg1 > FOUR_RING_THRESHOLD){
//            amount = RingAmount.FOUR;
//        }else if (avg1 > ONE_RING_THRESHOLD){
//            amount = RingAmount.ONE;
//        }else{
//            amount = RingAmount.NONE;
//        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                new Point(0, Y_MAX), // First point which defines the rectangle
                new Point(320, Y_MIN), // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }
}