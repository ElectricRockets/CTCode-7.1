package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IncrementLiftV2;
import org.firstinspires.ftc.teamcode.commands.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ScoreDuck;
import org.firstinspires.ftc.teamcode.commands.SetSmartIntakePower;
import org.firstinspires.ftc.teamcode.commands.ToggleHopper;
import org.firstinspires.ftc.teamcode.commands.ToggleTSEArm;
import org.firstinspires.ftc.teamcode.commands.ToggleTSEClaw;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp
public class DriverControlDel extends LinearOpMode {

    private double lastLoopTime = System.nanoTime() * Math.pow(10, -6);
    private GamepadEx mainGamepad;
    FFRobot robot;

    public void initialize() {
        mainGamepad = new GamepadEx(gamepad1);
        robot = new FFRobot(hardwareMap, telemetry);
        robot.initTele();

        //drives the robot
        robot.drive.setDefaultCommand(new MecanumDrive(
                robot.drive,
                mainGamepad::getLeftY,
                mainGamepad::getLeftX,
                mainGamepad::getRightX,
                robot.liftSubsystem::drivePower
        ));

        //automatically powers the intake at the specified power using the combination of 2 trigger inputs
        robot.intakeSubsystem.setDefaultCommand(new SetSmartIntakePower(
                robot.intakeSubsystem,
                () -> mainGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> mainGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                true
        ));

        //controls the lift and the hopper being open/closed
        mainGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IncrementLiftV2(robot.liftSubsystem));
        mainGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ToggleHopper(robot.liftSubsystem));
        mainGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_SHARED_CLOSED)));

        //automatic delivery of a duck for either playing field side depending on the button pressed
        mainGamepad.getGamepadButton(GamepadKeys.Button.X).whenActive(new ScoreDuck(
                robot.duckSubsystem,
                ScoreDuck.fieldSides.BLUE,
                ScoreDuck.scoreTypes.INSIDE
        ));

        mainGamepad.getGamepadButton(GamepadKeys.Button.B).whenActive(new ScoreDuck(
                robot.duckSubsystem,
                ScoreDuck.fieldSides.RED,
                ScoreDuck.scoreTypes.INSIDE
        ));

        //control inputs for the TSE arm and Claw
        mainGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ToggleTSEArm(robot.liftSubsystem));
        mainGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ToggleTSEClaw(robot.liftSubsystem));

        //toggles odometry being active
        //mainGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ToggleOdometry(robot.odometrySubsystem));
    }

    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
        }
        robot.reset();
    }
}
