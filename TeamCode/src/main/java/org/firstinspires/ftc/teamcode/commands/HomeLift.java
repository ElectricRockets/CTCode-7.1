package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class HomeLift extends CommandBase {

    LiftSubsystem liftSubsystem;
    private final double startTime;

    public HomeLift(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        startTime = System.nanoTime() * Math.pow(10,-9);
    }

    public void initialize() {
        liftSubsystem.setState(LiftSubsystem.states.HOMING);
    }

    public boolean isFinished() {
        return System.nanoTime() * Math.pow(10,-9) > startTime + 0.5;
    }

    public void end() {
        liftSubsystem.setState(LiftSubsystem.states.INTAKE);
    }
}
