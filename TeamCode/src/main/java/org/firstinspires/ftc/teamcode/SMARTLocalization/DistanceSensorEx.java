package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class DistanceSensorEx {

    private final Pose2d sensorPose;
    private final DistanceSensor sensor;
    public enum dSTypes {REV2M}
    private final dSTypes dSType;
    private final boolean onChub;

    public DistanceSensorEx(Pose2d sensorPose, dSTypes dSType, boolean onChub, DistanceSensor sensor) {
        this.sensorPose = sensorPose;
        this.sensor = sensor;
        this.dSType = dSType;
        this.onChub = onChub;
    }

    public double getDistance() {return sensor.getDistance(DistanceUnit.MM);}

    public double getVarianceEstimate(double distanceEstimate, double angleEstimate) {
        if (dSType == dSTypes.REV2M) {
            return Rev2mSensor.getVarianceEstimate(distanceEstimate, angleEstimate);
        }

        return Math.pow(2,31); //returns a nearly infinite value if the sensor type is unknown
    }

    public double getReadTime() {
        if (dSType == dSTypes.REV2M) {
            return Rev2mSensor.readTime(onChub);
        }
        return 0;
    }

    public Pose2d getSensorPose() {
        return sensorPose;
    }
}
