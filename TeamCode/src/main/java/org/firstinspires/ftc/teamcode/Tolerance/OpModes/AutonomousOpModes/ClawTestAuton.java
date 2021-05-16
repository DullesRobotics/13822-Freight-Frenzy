package org.firstinspires.ftc.teamcode.Tolerance.OpModes.AutonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Motor.Motor;
import org.firstinspires.ftc.teamcode.RobotManager.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Tolerance.Configurator;
import org.firstinspires.ftc.teamcode.Tolerance.Functions;

import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_END_TICKS;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_MID_TICKS;
import static org.firstinspires.ftc.teamcode.Tolerance.Functions.CLAW_MOTOR_PWR;

@Autonomous
public class ClawTestAuton extends LinearOpMode {

    MecanumDriveTrain robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecanumDriveTrain(this);
        robot.addHardware(Configurator.getHardware(robot));
        waitForStart();

        robot.autonWait(500);

        Motor m = robot.getMotor("CLM");
        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.FORWARD);
        m.get().setTargetPosition(CLAW_MOTOR_MID_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);
        robot.autonWait(2000);

        m.get().setPower(0);

        Functions.setClawServos(robot, true);

        robot.autonWait(1500);

        m.get().setPower(0);
        m.get().setDirection(DcMotorSimple.Direction.REVERSE);
        m.get().setTargetPosition(CLAW_MOTOR_END_TICKS);
        m.get().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.get().setPower(CLAW_MOTOR_PWR);

        robot.autonWait(800);

        m.get().setPower(0);

        robot.stopAllThreads();
        requestOpModeStop();
    }
}
