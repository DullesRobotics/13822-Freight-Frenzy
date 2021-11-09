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

import java.util.List;

public class GreenScanningPipeline extends OpenCvPipeline implements Pipeline {

    /* color constants */
    private static final Scalar BLUE = new Scalar(0, 0, 255), GREEN = new Scalar(0, 255, 0), RED = new Scalar(255, 0, 0);

    public static short Y_MAX = 30, Y_MIN = 200;
    public static short LEFT_2_MID_X = 107, MID_2_RIGHT_X = 214;

    private volatile Region selectedZone = Region.RIGHT;
    private volatile double leftWeight = 0, rightWeight = 0, middleWeight = 0;

    /*
     * Working variables
     */
    private Mat region1_Cb, region2_Cb, region3_Cb, HSV = new Mat(), Cb = new Mat();
    private int avg1;

    public enum Region {
        LEFT, MIDDLE, RIGHT
    }

    @Override
    public void updateLog(Logger l) {
        l.putData("Selected Zone", selectedZone);
        l.putData("Weights (L, M, R)", leftWeight + ", " + middleWeight + ", " + rightWeight);
    }

    void inputToCb(Mat input){


        //Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
       // Core.inRange(HSV, new Scalar(40,40,40), new Scalar(70,255,255), Cg);

        //Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        //inputToCb(firstFrame);

        region1_Cb = firstFrame.submat(new Rect(new Point(0, Y_MAX), new Point(320, Y_MIN)));
    }

    @Override
    public Mat processFrame(Mat img) {
        //inputToCb(input);
//        avg1 = (int) Core.mean(region1_Cb).val[0];
//
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                new Point(0, Y_MAX), // First point which defines the rectangle
//                new Point(320, Y_MIN), // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//
//        selectedZone = Region.MIDDLE;
////        if(avg1 > FOUR_RING_THRESHOLD){
////            amount = RingAmount.FOUR;
////        }else if (avg1 > ONE_RING_THRESHOLD){
////            amount = RingAmount.ONE;
////        }else{
////            amount = RingAmount.NONE;
////        }
//
        Imgproc.rectangle(
                img, // Buffer to draw on
                new Point(0, Y_MAX), // First point which defines the rectangle
                new Point(106, Y_MIN), // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        selectedZone = Region.LEFT;

        Imgproc.rectangle(
                img, // Buffer to draw on
                new Point(106, Y_MAX), // First point which defines the rectangle
                new Point(212, Y_MIN), // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                img, // Buffer to draw on
                new Point(212, Y_MAX), // First point which defines the rectangle
                new Point(320, Y_MIN), // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        int rows = img.rows(); //Calculates number of rows
        int cols = img.cols(); //Calculates number of columns
       // int ch = img.channels(); //Calculates number of channels (Grayscale: 1, RGB: 3, etc.)

        Point mostGreenPoint = new Point(0,0);
        double mostGreen = 0;

        for (int i=0; i<rows; i++)
        {
            for (int j=0; j<cols; j++)
            {
                double greenness = img.get(i,j)[1];
                if(greenness > mostGreen){
                    mostGreen = greenness;
                    mostGreenPoint = new Point(i, j);
                }
            }
        }
        Imgproc.rectangle(
                img, // Buffer to draw on
                new Point(mostGreenPoint.x+25, mostGreenPoint.y+25), // First point which defines the rectangle
                new Point(mostGreenPoint.x-25, mostGreenPoint.y-25), // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        return img;
    }
}