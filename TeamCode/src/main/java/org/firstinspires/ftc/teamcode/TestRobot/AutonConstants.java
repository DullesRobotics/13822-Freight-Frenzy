package org.firstinspires.ftc.teamcode.TestRobot;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * All numbers in degrees or inches
 */
@Config
public class AutonConstants {

    public static OpenCvCameraRotation OPEN_CV_CAM_ROTATION = OpenCvCameraRotation.UPRIGHT;

    //x is y and y is -x :(

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
    public static double START_STACK_ANGLE = 20;

    /**
     * The points at the center of each zone. <br>
     * X points reversed for red
     */
    public static Point
            ZONE_A = new Point(-58, -5),
            ZONE_B = new Point(-34, 19),
            ZONE_C = new Point(-58, 43);

    /**
     * The offsets from the zone points
     * for the first and second wobble goal to give space. <br>
     * will be reversed for red
     */
    public static double
            FIRST_WOBBLE_OFFSET_X = 0,
            FIRST_WOBBLE_OFFSET_Y = 0,
            SECOND_WOBBLE_OFFSET_X = 0,
            SECOND_WOBBLE_OFFSET_Y = 0;

    /**
     * Inches to move back before raising claw
     */
    public static double WOBBLE_SAFE_CLAW_ARM_DISTANCE = 4;

    /** Points to shoot from */
    public static Point
        SHOOTING_POSITION_BLUE = new Point(-34, -2);

    /** Angles to shoot from */
    public static double
        SHOOTING_ANGLE_BLUE = -6;

    public static double ANGLE_TO_CHANGE = -5;
    public static double POWER_TO_CHANGE = 0.03;

            public static double SHOOTER_POWER = -0.7;

    public static Point
        BLUE_RETURN_POINT_1 = new Point(0, SHOOTING_POSITION_BLUE.y),
        BLUE_RETURN_POINT_2 = new Point(BLUE_RETURN_POINT_1.x, -52),
        BLUE_RETURN_POINT_3 = new Point(-24, BLUE_RETURN_POINT_2.y);

    public static double INCHES_FORWARD_FOR_WOBBLE_PICKUP = 3;

    public static double LAUNCH_LINE_Y_COORDINATE = 12;

}
