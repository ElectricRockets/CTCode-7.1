package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

@TeleOp
public class DriverControlDual extends LinearOpMode {

    private GamepadEx mainGamepad;
    private GamepadEx secondaryGamepad;
    FFRobot robot;

    public void initialize() {
        mainGamepad = new GamepadEx(gamepad1);
        secondaryGamepad = new GamepadEx(gamepad2);
        robot = new FFRobot(this);
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
        secondaryGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IncrementLiftV2(robot.liftSubsystem)
        );
        secondaryGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ToggleHopper(robot.liftSubsystem)
        );
        secondaryGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(
                () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_SHARED_CLOSED)
        ));

        //automatic delivery of a duck for either playing field side depending on the button pressed
        secondaryGamepad.getGamepadButton(GamepadKeys.Button.X).whenActive(new ConditionalCommand(
                new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.BLUE, ScoreDuck.scoreTypes.OUTSIDE),
                new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.BLUE, ScoreDuck.scoreTypes.INSIDE),
                () -> mainGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ));

        secondaryGamepad.getGamepadButton(GamepadKeys.Button.B).whenActive(new ConditionalCommand(
                new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.RED, ScoreDuck.scoreTypes.OUTSIDE),
                new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.RED, ScoreDuck.scoreTypes.INSIDE),
                () -> mainGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ));

        //control inputs for the TSE arm and Claw
        mainGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ToggleTSEArm(robot.liftSubsystem));
        mainGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ToggleTSEClaw(robot.liftSubsystem));

        //toggles odometry being active
        //mainGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ToggleOdometry(robot.odometrySubsystem));
    }

    public void runOpMode() {
        initialize();

        robot.waitForStartTele();

        // run the scheduler
        robot.loop();

        robot.reset();
    }
}
