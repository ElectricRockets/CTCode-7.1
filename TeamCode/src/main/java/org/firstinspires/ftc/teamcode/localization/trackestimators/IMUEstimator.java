package org.firstinspires.ftc.teamcode.localization.trackestimators;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.localization.base.TrackEstimator;

import java.util.function.Supplier;

public class IMUEstimator implements TrackEstimator {

    public enum axis {X,Y,Z}
    private final axis rotationAxis;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final BNO055IMU imu;

    private double lastReading;

    public IMUEstimator(Supplier<Pose2d> robotPoseSupplier, BNO055IMU imu, axis rotationAxis) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.imu = imu;
        this.rotationAxis = rotationAxis;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void update() {
        switch (rotationAxis) {
            case X: lastReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
                break;
            case Y: lastReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
                break;
            case Z: lastReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
                break;
        }
    }

    @Override
    public boolean getValidityRotation() {
        return true;
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d( robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), new Rotation2d(lastReading));
    }
}
