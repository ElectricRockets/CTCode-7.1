package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.DuckSubsystem;

import java.util.function.Supplier;


public class ScoreDuck extends CommandBase {
    private final DuckSubsystem DuckSubsystem;
    private double startTime;
    public enum fieldSides {RED, BLUE}
    public enum scoreTypes {INSIDE, OUTSIDE, UNIVERSAL}
    private double accel;
    private double maxSpeed;
    private double time;
    private final double powerSign;

    public ScoreDuck(DuckSubsystem duckSubsystem, fieldSides fieldSide, Supplier<scoreTypes> scoreType) {
        DuckSubsystem = duckSubsystem;
        addRequirements(duckSubsystem);

        switch (scoreType.get()) {
            case INSIDE:
                accel = RobotConstants.INSIDE_DELIVERY_ACCEL;
                maxSpeed = RobotConstants.INSIDE_DELIVERY_MAX_SPEED;
                time = RobotConstants.INSIDE_DELIVERY_TIME;
                break;
            case OUTSIDE:
                accel = RobotConstants.OUTSIDE_DELIVERY_ACCEL;
                maxSpeed = RobotConstants.OUTSIDE_DELIVERY_MAX_SPEED;
                time = RobotConstants.OUTSIDE_DELIVERY_TIME;
                break;
            case UNIVERSAL:
                accel = RobotConstants.UNIVERSAL_DELIVERY_ACCEL;
                maxSpeed = RobotConstants.UNIVERSAL_DELIVERY_MAX_SPEED;
                time = RobotConstants.UNIVERSAL_DELIVERY_TIME;
                break;
        }

        powerSign = fieldSide == fieldSides.RED ? 1 : -1;
    }

    public ScoreDuck(DuckSubsystem duckSubsystem, fieldSides fieldSide, scoreTypes scoreType) {
        this(duckSubsystem, fieldSide, () -> scoreType);
    }

    public void initialize() {
        startTime = System.nanoTime() * Math.pow(10,-9);
        DuckSubsystem.spinBlue(0);
    }

    public void execute() {
        DuckSubsystem.setPower( powerSign * Math.min(
                ((System.nanoTime() * Math.pow(10,-9)) - startTime) * accel + RobotConstants.BASE_DUCK_POWER,
                maxSpeed
        ));
    }

    public boolean isFinished() {
        return (startTime + time <= (System.nanoTime() * Math.pow(10,-9)));
    }

    public void end(boolean interrupted) {
        DuckSubsystem.spinBlue(0);
    }
}
