package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class ToggleTSEClaw extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private double startTime;
    private boolean releaseMode = false;

    public ToggleTSEClaw(LiftSubsystem liftSubsystem) {

        this.liftSubsystem = liftSubsystem;
    }

    public void initialize() {
        startTime = System.nanoTime() * Math.pow(10,-9);

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
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_LOW_CLOSED_HOPPER_CLOSED);
                releaseMode = true;
                break;
            case SCORE_TSE_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_CLOSED);
                break;
            case SCORE_TSE_CLOSED_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_LOW_CLOSED_HOPPER_OPEN);
                releaseMode = true;
                break;
            case SCORE_TSE_OPEN_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_CLOSED_HOPPER_OPEN);
                break;
        }
    }

    public void execute() {
        if (System.nanoTime() * Math.pow(10,-9) > startTime + RobotConstants.SCORE_TSE_RELEASE_DELAY) {
            switch (liftSubsystem.state) {
                case SCORE_TSE_LOW_CLOSED_HOPPER_CLOSED:
                    liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_LOW_OPEN_HOPPER_CLOSED);
                    break;
                case SCORE_TSE_LOW_CLOSED_HOPPER_OPEN:
                    liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_LOW_OPEN_HOPPER_OPEN);
                    break;
            }
        }
    }

    public boolean isFinished() {
        if (releaseMode) {
            return System.nanoTime() * Math.pow(10,-9) > startTime + RobotConstants.SCORE_TSE_DELAY;
        }
        return true;
    }

    public void end(boolean isInterrupted) {
        switch(liftSubsystem.state) {
            case SCORE_TSE_LOW_OPEN_HOPPER_OPEN:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_OPEN);
                break;
            case SCORE_TSE_LOW_OPEN_HOPPER_CLOSED:
                liftSubsystem.setState(LiftSubsystem.states.SCORE_TSE_OPEN_HOPPER_CLOSED);
                break;
        }
    }
}
