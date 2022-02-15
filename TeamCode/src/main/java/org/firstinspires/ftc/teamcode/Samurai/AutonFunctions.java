package org.firstinspires.ftc.teamcode.Samurai;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ComponentArea;
import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;

@Config
public class AutonFunctions {
    private static volatile MecanumDriveTrain mainFrame;
    private static volatile SampleMecanumDrive roadRunner;
    private static int ticksGround = 0, ticksLevel1 = 333, ticksLevel2 = 666, ticksLevel3 = 1000;
    private final static long timeToDriveFar = 1000, timeToDriveNear = 900;

    public static void start(LinearOpMode op, TeamColor t, FieldPosition position){
        mainFrame = new MecanumDriveTrain(op);
        mainFrame.addHardware(Configurator.getHardware(mainFrame));

        op.waitForStart();
        if(op.isStopRequested()) return;
        long timeToDrive = position == FieldPosition.NEAR_CAROUSEL ? timeToDriveFar : timeToDriveNear;
        timeToDrive += System.currentTimeMillis();
        while(System.currentTimeMillis() < timeToDrive && op.opModeIsActive())
            mainFrame.setSidedDrivePower(1, 1);
        mainFrame.setSidedDrivePower(0,0);
        op.requestOpModeStop();
    }

    public static void startNew(LinearOpMode op, TeamColor t, FieldPosition position){
        roadRunner = new SampleMecanumDrive(op);
        mainFrame = roadRunner.getDriveTrain();

        ///////////

        Trajectory moveToWobble = roadRunner.trajectoryBuilder(new Pose2d())
                .forward(12)
                .build();

        Trajectory moveToCarousel = roadRunner.trajectoryBuilder(new Pose2d())
                .strafeRight(12)
                .build();

        Trajectory parkInStorage = roadRunner.trajectoryBuilder(new Pose2d())
                .forward(16)
                .build();

        Trajectory parkInWarehouse = roadRunner.trajectoryBuilder(new Pose2d())
                .forward(16)
                .build();

        ///////////

        resetLiftEncoder();

        // scan barcode

        // determine level
        int level = 1;

        op.waitForStart();

        if(op.isStopRequested()) return;

        // move lift to correct level
        changeLiftLevel(level);

        // move to middle wobble
        roadRunner.followTrajectory(moveToWobble);

        // wait for lift to extend
        mainFrame.autonWait(1000);

        // place block
        intakeItems(false, true);

        // wait a few seconds to drop
        mainFrame.autonWait(500);

        // turn off intake
        intakeItems(false, false);

        // lower intake
        changeLiftLevel(0);

        // IF NEAR CAROUSEL
        if(position == FieldPosition.NEAR_CAROUSEL) {
            // move to carousel
            roadRunner.followTrajectory(moveToCarousel);
            // spin 5 seconds
            spinCarousel(true);
            mainFrame.autonWait(5000);
            spinCarousel(false);
            // park Dropoff
            roadRunner.followTrajectory(parkInStorage);
        } else {
            // ELSE NEAR WAREHOUSE
            // park warehouse
            roadRunner.turn(Math.toRadians(90));
            roadRunner.followTrajectory(parkInWarehouse);
        }
    }

    /**
     * -1 -> Motor moving up
     * 1 -> Motor moving down
     * @param isOn - turns the motor based on direction
     * @param dir - tells the direction of the motor that controls intake
     **/
    public void intakeItems (boolean dir, boolean isOn){
        Motor container = mainFrame.getMotors(ComponentArea.INTAKE).get(0);
        if(container != null && container.get() != null)
            if(isOn == true)
                container.get().setPower(forward ? -1 : 1);
            else
                container.get().setPower(0);
    }


    /**
     * 0 - ground
     * 1 - level 1
     * 2 - level 2
     * 3 - level 3
     * 4 - level 4
     * @param level The level to move the lift to
     */
    public static void changeLiftLevel(int level){
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

    public static void resetLiftEncoder(){
        Motor liftMotor = mainFrame.getMotors(ComponentArea.LIFT).get(0);
        if(liftMotor != null && liftMotor.isEncoded() && liftMotor.getEncoded() != null)
            liftMotor.stopAndResetEncoder();
    }

    /**
     * Sets if the carousel should spin or not
     * @param isOn True - Carousel Spins; False - Carousel Stops
     */
    public static void spinCarousel (boolean isOn) {
        Motor carousel = mainFrame.getMotors(ComponentArea.CAROUSEL).get(0);
        if(carousel != null && carousel.get() != null)
            carousel.get().setPower(isOn ? 1 : 0);
    }

    public enum TeamColor {
        RED, BLUE
    }

    public enum FieldPosition {
        NEAR_WAREHOUSE, NEAR_CAROUSEL
    }

}

