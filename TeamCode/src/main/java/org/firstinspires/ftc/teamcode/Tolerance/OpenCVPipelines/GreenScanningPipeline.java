package org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines;

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
  //  private Mat yCrCB;
    private Mat channel;
    private Mat threshold;
    private List<MatOfPoint> contours;

    private Rect rect;
    private Point currentPoint;

    public GreenScanningPipeline() {
        //yCrCB = new Mat();
        channel = new Mat();
        threshold = new Mat();
        contours = new ArrayList<>();
        rect = new Rect();
    }

    @Override
    public Mat processFrame(Mat input) {
        currentPoint = getCenterOfRect(detectHighGoal(input));
        return input;
    }

    private Rect detectHighGoal(Mat input) {
        contours.clear();
//        Imgproc.cvtColor(input, yCrCB, Imgproc.COLOR_RGB2YCrCb);
        Scalar annotationColor;

        //coi 1 for red, 2 for blue
        Core.extractChannel(/*yCrCB*/ input, channel, 1);
        // red green blue
        annotationColor = new Scalar(0, 255, 0);


        Imgproc.threshold(channel, threshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = contours.size() - 1; i >= 0; i--) {
            Rect sampleRect = Imgproc.boundingRect(contours.get(i));
            double ratio = (double) sampleRect.height / sampleRect.width;
            double ratioInverse = (double) sampleRect.width / sampleRect.height;
            if (ratio > 2 && ratioInverse < 1) {
                contours.remove(i);
            }
        }

        int maxVal = 0;
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            rect = Imgproc.boundingRect(contours.get(i));
            Imgproc.rectangle(input, rect, new Scalar(255, 255, 255), 1);
            if (maxVal < rect.height) {
                maxVal = rect.height;
                index = i;
            }
        }
        if (contours.size() == 0) rect = null;
        else {
            rect = Imgproc.boundingRect(contours.get(index));
            Imgproc.rectangle(input, rect, annotationColor, 3);
            Imgproc.rectangle(input, new Rect(1,2,3,4), annotationColor, 3);
        }

        return rect;
    }

    public Point getCenterOfRect(Rect rect) {
        if (rect == null) {
            return new Point(-1000, -1000);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public Zone getZone() {
        if(getCenterOfRect(rect).x < Zone.LEFT.maxX)
            return Zone.LEFT;
        else if(getCenterOfRect(rect).x < Zone.MIDDLE.maxX)
            return Zone.MIDDLE;
        else return Zone.RIGHT;
    }

    public boolean isObjectVisible() {
        return rect != null;
    }

    public Point getCamPoint() {
        return currentPoint;
    }

    @Override
    public void updateLog(Logger l) {
        l.putData("Object Visible", isObjectVisible());
        l.putData("Point: ", currentPoint);
        l.putData("Zone: ", getZone().name());
    }

    enum Zone {
        LEFT(0, 106),
        MIDDLE(107, 212),
        RIGHT(213,320);

        private int minX, maxX;

        Zone (int minX, int maxX){
            this.maxX = maxX;
            this.minX = minX;
        }
    }
}