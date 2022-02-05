package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
    public DcMotorEx intake;
    public Servo ramp;
    public enum states {RETRACTED, INTAKE, EXTAKE, SMART_INTAKE}
    public BooleanSupplier intakeAllowed;
    public BooleanSupplier autoExtakeAllowed;
    public states state;

    public IntakeSubsystem(HardwareMap hardwareMap, BooleanSupplier intakeAllowed, BooleanSupplier autoExtakeAllowed) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ramp = hardwareMap.get(Servo.class, "intakeRamp");
        this.intakeAllowed = intakeAllowed;
        this.autoExtakeAllowed = autoExtakeAllowed;
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        state = states.RETRACTED;
        intake.setCurrentAlert(4, CurrentUnit.AMPS);
    }

    public void setState(states newState) {
        state = newState;
        switch (state) {
            case RETRACTED:
                setIntakePower(0);
                break;
            case INTAKE:
                setIntakePower(RobotConstants.INTAKE_POWER_DEFAULT);
                break;
            case EXTAKE:
                setIntakePower(RobotConstants.EXTAKE_POWER_DEFAULT);
        }
    }


    public void periodic() {

        if (state == states.SMART_INTAKE) {
            ramp.setPosition(RobotConstants.INTAKE_RAMP_LOW);
            if (intakeAllowed.getAsBoolean()) {
                intake.setPower(RobotConstants.INTAKE_POWER_DEFAULT);
            } else {
                intake.setPower(RobotConstants.EXTAKE_POWER_DEFAULT);
            }
        }
    }

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
        if (intakePower == 0) {
            ramp.setPosition(RobotConstants.INTAKE_RAMP_HIGH);
        } else {
            ramp.setPosition(RobotConstants.INTAKE_RAMP_LOW);
        }
    }

    public boolean isStalled() {
        return intake.isOverCurrent();
    }
}
