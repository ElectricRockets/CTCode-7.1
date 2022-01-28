package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class IncrementLiftV2 extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public IncrementLiftV2(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        switch (liftSubsystem.state) {
            case INTAKE:
                liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED);
                break;
            case GRAB_TSE_OPEN_INTAKE_OPEN:
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
            case GRAB_TSE_OPEN_INTAKE_CLOSED:
            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
            case INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_SHARED_CLOSED);
                break;
            case SCORE_SHARED_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED);
                break;
            case SCORE_HIGH_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN);
                break;
            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_OPEN);
                break;
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_OPEN);
                break;
            case SCORE_HIGH_OPEN:
            case SCORE_SHARED_OPEN:
            case SCORE_TSE_OPEN_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.INTAKE);
                break;
        }
    }

    public boolean isFinished() {
        return true;
    }
}
