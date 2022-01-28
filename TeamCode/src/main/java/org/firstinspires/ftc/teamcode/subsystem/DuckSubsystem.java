package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSubsystem extends SubsystemBase {
    public CRServo leftDuck;
    public CRServo rightDuck;
    InterpLUT servoSpeeds = new InterpLUT();

    public DuckSubsystem(HardwareMap hardwareMap) {
        leftDuck = hardwareMap.get(CRServo.class, "leftDuck");
        rightDuck = hardwareMap.get(CRServo.class, "rightDuck");
        servoSpeeds.add(0.0,0.0);
        servoSpeeds.add(1.0,1.0);
        servoSpeeds.createLUT();
        setPower(0);
    }

    public void periodic() {
    }

    public void setPower(double power) {
        leftDuck.setPower(power);
        rightDuck.setPower(power);
    }

    public void spinRed(double inputPower) {
        setPower(inputPower);
    }

    public void spinBlue(double inputPower) {
        setPower(-inputPower);
    }
}
