package org.firstinspires.ftc.teamcode.Tolerance.OpenCVPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.AddOns.Pipeline;
import org.firstinspires.ftc.teamcode.Libraries.Logger;
import org.firstinspires.ftc.teamcode.Tolerance.AutonRunner;
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

import static org.firstinspires.ftc.teamcode.Tolerance.AutonRunner.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Tolerance.AutonRunner.Team.RED;

/**
 * This class is courtesy of Wizards.exe
 */
@Config
public class HighGoalDetectionPipeline extends OpenCvPipeline implements Pipeline {

        public int minThreshold = 155;
        public int maxThreshold = 255;
        private Mat yCrCB;
        private Mat channel;
        private Mat threshold;
        private List<MatOfPoint> contours;

        private Rect rect;
        private Point currentPoint;

        public HighGoalDetectionPipeline() {
            yCrCB = new Mat();
            channel = new Mat();
            threshold = new Mat();
            contours = new ArrayList<>();
            rect = new Rect();
        }

        @Override
        public Mat processFrame(Mat input) {
            currentPoint = getCenterOfRect(detectHighGoal(input, AutonRunner.team));
            return input;
        }

        private Rect detectHighGoal(Mat input, AutonRunner.Team color) {
            contours.clear();
            Imgproc.cvtColor(input, yCrCB, Imgproc.COLOR_RGB2YCrCb);
            Scalar annotationColor;
            if (color == RED) {
                Core.extractChannel(yCrCB, channel, 1);
                annotationColor = new Scalar(255, 0, 0);
            } else {
                Core.extractChannel(yCrCB, channel, 2);
                annotationColor = new Scalar(0, 0, 255);
            }

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
            }

            return rect;
        }

        public Point getCenterOfRect(Rect rect) {
            if (rect == null) {
                return new Point(-1000, -1000);
            }
            return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }

        public boolean isGoalVisible() {
            return rect != null;
        }

        public Point getCamPoint() {
            return currentPoint;
        }

    @Override
    public void updateLog(Logger l) {
        l.putData("GoalVisible", isGoalVisible());
        l.putData("Team " + AutonRunner.team.toString() + " Point: ", currentPoint);
    }

}
