package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class SetSmartIntakePower extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier intakeInput;
    private final DoubleSupplier extakeInput;
    private boolean autoMaxExtake;

    public SetSmartIntakePower(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeInput, DoubleSupplier extakeInput, boolean autoMaxExtake) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeInput = intakeInput;
        this.extakeInput = extakeInput;
        this.autoMaxExtake = autoMaxExtake;
        addRequirements(intakeSubsystem);
    }

    public void execute() {
        double intakeInputPower = intakeSubsystem.intakeAllowed.getAsBoolean() ? intakeInput.getAsDouble() * RobotConstants.INTAKE_POWER_DEFAULT :
                (intakeSubsystem.autoExtakeAllowed.getAsBoolean() && autoMaxExtake ? RobotConstants.EXTAKE_POWER_DEFAULT : intakeInput.getAsDouble() * RobotConstants.EXTAKE_POWER_DEFAULT);
        double extakeInputPower = extakeInput.getAsDouble() * RobotConstants.EXTAKE_POWER_DEFAULT;
        intakeSubsystem.setIntakePower(intakeInputPower + extakeInputPower);
    }
}
