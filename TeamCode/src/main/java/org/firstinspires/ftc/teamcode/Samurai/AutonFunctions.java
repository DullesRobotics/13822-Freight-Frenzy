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
    private static int ticksGround = 0, ticksLevel1 = 333, ticksLevel2 = 666, ticksLevel3 = 1000;

    public static void start(LinearOpMode op, TeamColor t, Direction position){

    }

    // intake in/out/off


    /**
     * 0 - ground
     * 1 - level 1
     * 2 - level 2
     * 3 - level 3
     * 4 - level 4
     * @param level The level to move the lift to
     */
    public void changeLiftLevel(int level){
        Motor liftMotor = mainFrame.getMotors(ComponentArea.LIFT).get(0);
        if(liftMotor != null && liftMotor.isEncoded() && liftMotor.getEncoded() != null){
            int levelTicks = 0;
            switch(level){
                case 1: levelTicks = ticksLevel1; break;
                case 2: levelTicks = ticksLevel2; break;
                case 3: levelTicks = ticksLevel3; break;
                default:
                case 0: levelTicks = ticksGround; break;
            }
            liftMotor.getEncoded().setTargetPosition(levelTicks);
        }
    }

    public void resetLift

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

