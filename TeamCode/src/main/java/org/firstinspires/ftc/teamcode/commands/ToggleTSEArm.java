package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class ToggleTSEArm extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public ToggleTSEArm(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    public void execute() {
        switch (liftSubsystem.state) {
            case INTAKE:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_OPEN);
                break;
            case INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED);
                break;
            case GRAB_TSE_CLOSED_INTAKE_OPEN:
            case GRAB_TSE_OPEN_INTAKE_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.INTAKE);
                break;
            case GRAB_TSE_CLOSED_INTAKE_CLOSED:
            case GRAB_TSE_OPEN_INTAKE_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED);
                break;
            case SCORE_HIGH_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_CLOSED);
                break;
            case SCORE_HIGH_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_OPEN);
                break;
            case SCORE_TSE_OPEN_HOPPER_OPEN:
            case SCORE_TSE_CLOSED_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN);
                break;
            case SCORE_TSE_CLOSED_HOPPER_CLOSED:
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED);
                break;
        }
    }

    public boolean isFinished() {
        return true;
    }
}
