package org.firstinspires.ftc.teamcode.TestRobot.OpenCVPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.Pipeline;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This class is courtesy of Wizards.exe
 */
@Config
public class UltimateGoalPipeline extends OpenCvPipeline implements Pipeline {

    public enum RingAmount {
        NONE, ONE, FOUR
    }

    /* Some color constants */
    private static final Scalar BLUE = new Scalar(0, 0, 255), GREEN = new Scalar(0, 255, 0);

    /* The core values which define the location and size of the sample regions */
    public static Point region1_pointA = new Point(50,135);
    public static Point region1_pointB = new Point(250, 230);

    /* Values that determine rings. Guess & check until it works */
    public static int FOUR_RING_THRESHOLD = 130, ONE_RING_THRESHOLD = 126;

    /*
     * Working variables
     */
    private Mat region1_Cb, YCrCb = new Mat(), Cb = new Mat();
    private int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile RingAmount amount = RingAmount.FOUR;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     * Basically removes the blue
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        amount = RingAmount.FOUR; // Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            amount = RingAmount.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            amount = RingAmount.ONE;
        }else{
            amount = RingAmount.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis()
    {
        return avg1;
    }

    public RingAmount getAmount(){
        return amount;
    }

    @Override
    public void updateLog(Logger l) {
        l.putData("Analysis", getAnalysis());
        l.putData("Position", getAmount().toString());
    }

}
