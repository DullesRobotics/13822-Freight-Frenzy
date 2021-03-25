package org.firstinspires.ftc.teamcode.TestRobot;

import android.graphics.Point;

import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * All numbers in degrees or inches
 */
public class AutonConstants {

    public static OpenCvCameraRotation OPEN_CV_CAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    /**
     * Starting positions on the blue/red lines
     */
    public static Point
            STARTING_BLUE_LEFT = new Point(-48, -60),
            STARTING_BLUE_RIGHT = new Point(-24, -60),
            STARTING_RED_LEFT = new Point(24, -60),
            STARTING_RED_RIGHT = new Point(48, -60);

    /**
     * Angle to turn from the starting position
     * to see the starting stack
     * IN DEGREES
     */
    public static double START_STACK_ANGLE = 18.435;

    /**
     * The points at the center of each zone. <br>
     * X points reversed for red
     */
    public static Point
            ZONE_A = new Point(-60, 12),
            ZONE_B = new Point(-36, 36),
            ZONE_C = new Point(-60, 60);

    /**
     * The offsets from the zone points
     * for the first and second wobble goal to give space. <br>
     * will be reversed for red
     */
    public static double
            FIRST_WOBBLE_OFFSET = -6,
            SECOND_WOBBLE_OFFSET = 6;

    /**
     * Inches to move back before raising claw
     */
    public static double WOBBLE_SAFE_CLAW_ARM_DISTANCE = 4;

    /** Points to shoot from */
    public static Point
        SHOOTING_POSITION_BLUE = new Point(-58, 6);

    /** Angles to shoot from */
    public static double
        SHOOTING_ANGLE_BLUE = 12;

    public static Point
        BLUE_RETURN_POINT_1 = new Point(0, SHOOTING_POSITION_BLUE.y),
        BLUE_RETURN_POINT_2 = new Point(BLUE_RETURN_POINT_1.x, -52),
        BLUE_RETURN_POINT_3 = new Point(-24, BLUE_RETURN_POINT_2.y);

    public static double INCHES_FORWARD_FOR_WOBBLE_PICKUP = 3;

    public static double LAUNCH_LINE_Y_COORDINATE = 12;

}
