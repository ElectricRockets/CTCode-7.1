package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDrive extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX, powerScale;

    public MecanumDrive(MecanumDriveSubsystem drive, DoubleSupplier leftY,
                        DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier powerScale) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.powerScale = powerScale;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.update();
        drive.driveWithInput(-leftY.getAsDouble() * powerScale.getAsDouble(), leftX.getAsDouble() * powerScale.getAsDouble(), rightX.getAsDouble() * powerScale.getAsDouble());
    }

}