package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class OdometrySubsystem extends SubsystemBase {
    private final Servo odoRetraction;
    private final MecanumDriveSubsystem drive;
    public enum states {RETRACTED, DEPLOYED}
    public states state;
    public double lastStateChange;
    //private final Localizer defaultLocalizer;
    //private final Localizer odometryLocalizer;

    public OdometrySubsystem(HardwareMap hardwareMap, MecanumDriveSubsystem drive) {
        this.drive = drive;
        odoRetraction = hardwareMap.get(Servo.class, "odoRetraction");
        state = states.DEPLOYED;
        lastStateChange = System.nanoTime() * Math.pow(10,-9);
        //defaultLocalizer = sampleMecanumDrive.getLocalizer();
        //odometryLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
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
