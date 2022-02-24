package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;

public class OdometrySubsystem extends SubsystemBase {
    private final Servo odoRetraction;
    private final MecanumDriveSubsystem drive;
    public enum states {RETRACTED, DEPLOYED}
    public states state;
    public double lastStateChange;

    public OdometrySubsystem(HardwareMap hardwareMap, Telemetry telemetry, MecanumDriveSubsystem drive) {
        this.drive = drive;
        odoRetraction = hardwareMap.get(Servo.class, "odoRetraction");
        state = states.DEPLOYED;
        lastStateChange = System.nanoTime() * Math.pow(10,-9);
    }

    public void setState(states newState) {
        state = newState;
        switch(state) {
            case DEPLOYED:
                odoRetraction.setPosition(RobotConstants.ODOMETRY_DEPLOYED);
                //drive.setLocalizer(odometryLocalizer);
                break;
            case RETRACTED:
                odoRetraction.setPosition(RobotConstants.ODOMETRY_RETRACTED);
                //drive.setLocalizer(defaultLocalizer);
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
