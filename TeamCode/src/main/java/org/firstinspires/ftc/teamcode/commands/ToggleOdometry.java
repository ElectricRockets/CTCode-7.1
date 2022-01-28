package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.OdometrySubsystem;

public class ToggleOdometry extends CommandBase {
    private final OdometrySubsystem odometrySubsystem;

    public ToggleOdometry(OdometrySubsystem odometrySubsystem) {
        this.odometrySubsystem = odometrySubsystem;
    }

    public void execute() {
        if (odometrySubsystem.state == OdometrySubsystem.states.DEPLOYED) {
            odometrySubsystem.setState(OdometrySubsystem.states.RETRACTED);
        } else {
            odometrySubsystem.setState(OdometrySubsystem.states.DEPLOYED);
        }
    }

    public boolean isFinished() {
        return true;
    }
}
