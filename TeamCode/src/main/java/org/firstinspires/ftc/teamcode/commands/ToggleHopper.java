package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class ToggleHopper extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public ToggleHopper(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    public void execute() {
        switch (liftSubsystem.state) {
            case SCORE_LOW_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN);
                break;
            case SCORE_SHARED_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_SHARED_OPEN);
                break;
            case SCORE_MID_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_OPEN);
                break;
            case SCORE_HIGH_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN);
                break;
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_OPEN);
                break;
            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_OPEN);
                break;
            case INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.INTAKE);
                break;
        }
        liftSubsystem.needsToDeregister = true;
    }

    public boolean isFinished() {
        return true;
    }
}
