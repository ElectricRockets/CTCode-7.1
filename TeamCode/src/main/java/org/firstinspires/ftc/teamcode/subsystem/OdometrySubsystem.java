package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;

public class OdometrySubsystem extends SubsystemBase {
    private final Servo odoRetraction;
    public enum states {RETRACTED, DEPLOYED}
    public states state;
    public double lastStateChange;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        odoRetraction = hardwareMap.get(Servo.class, "odoRetraction");
        state = states.DEPLOYED;
        lastStateChange = System.nanoTime() * Math.pow(10,-9);
    }

    public void setState(states newState) {
        state = newState;
        switch(state) {
            case DEPLOYED:
                odoRetraction.setPosition(RobotConstants.ODOMETRY_DEPLOYED);
                break;
            case RETRACTED:
                odoRetraction.setPosition(RobotConstants.ODOMETRY_RETRACTED);
                break;
        }
    }

    public boolean odometryDeployed() {
        return state == states.DEPLOYED;
    }

    public boolean odometryActive() {
        return state == states.DEPLOYED && System.nanoTime() * Math.pow(10,-9) - lastStateChange > RobotConstants.ODOMETRY_DEPLOY_WAIT;
    }
}
