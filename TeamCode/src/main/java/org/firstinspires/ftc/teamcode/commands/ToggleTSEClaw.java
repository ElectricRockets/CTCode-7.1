package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class ToggleTSEClaw extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public ToggleTSEClaw(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    public void execute() {
        switch (liftSubsystem.state) {
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_OPEN);
                break;
            case GRAB_TSE_OPEN_INTAKE_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_OPEN);
                break;
            case GRAB_TSE_OPEN_INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED);
                break;
            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED);
                break;
            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_CLOSED);
                break;
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_CLOSED);
                break;
            case SCORE_TSE_CLOSED_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_OPEN);
                break;
            case SCORE_TSE_OPEN_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_OPEN);
        }
    }

    public boolean isFinished() {
        return true;
    }
}
