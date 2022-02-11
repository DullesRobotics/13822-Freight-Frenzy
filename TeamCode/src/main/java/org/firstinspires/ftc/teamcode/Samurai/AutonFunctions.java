package org.firstinspires.ftc.teamcode.Samurai;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

@Config
public class AutonFunctions {
    private static volatile MecanumDriveTrain mainFrame;
    private static volatile SampleMecanumDrive roadRunner;
    private static long ticksGround = 0, ticksLevel1 = 333, ticksLevel2 = 666, ticksLevel3 = 1000;

    public static void start(LinearOpMode op, TeamColor t, Direction position){

    }

    // intake in/out/off
    public void intakeItems (boolean dir, boolean isOn){
        Motor container = mainFrame.getMotors(ComponentArea.INTAKE).get(0);
        if(container != null && container.get() != null)
            if(isOn == true)
                container.get().setPower(dir ? -1 : 1);
            else
                container.get().setPower(0);
    }

    // lift level 0/1/2/3


    /**
     * Sets if the carousel should spin or not
     * @param isOn True - Carousel Spins; False - Carousel Stops
     */
    public void spinCarousel (boolean isOn) {
        Motor carousel = mainFrame.getMotors(ComponentArea.CAROUSEL).get(0);
        if(carousel != null && carousel.get() != null)
            carousel.get().setPower(isOn ? 1 : 0);
    }

    public enum TeamColor {
        RED, BLUE
    }

    public enum Direction {
        LEFT, RIGHT
    }

}

