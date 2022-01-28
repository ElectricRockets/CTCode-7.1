package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class SetIntakePower extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier power;

    public SetIntakePower(IntakeSubsystem IntakeSubsystem, DoubleSupplier power) {
        this.power = power;
        intakeSubsystem = IntakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakePower(power.getAsDouble());
    }
}
