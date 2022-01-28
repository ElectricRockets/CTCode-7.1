package org.firstinspires.ftc.teamcode.localization.trackupdater;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.localization.base.TrackUpdater;
import org.firstinspires.ftc.teamcode.localization.trackestimators.IMUEstimator;

import java.lang.reflect.Array;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class MecanumUpdater implements TrackUpdater {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final BooleanSupplier validity;
    private final DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private final double trackWidth, wheelRadius, ticksPerRev, xMultiplier, yMultiplier;
    private int rightFrontPreviousPosition, leftFrontPreviousPosition, rightRearPreviousPosition,leftRearPreviousPosition;
    private Pose2d localChange;
    private final BNO055IMU imu;
    public enum axis {X,Y,Z}
    private final axis rotationAxis;

    private final double xVariance;
    private final double yVariance;
    private final double headingVariance;

    public MecanumUpdater(Supplier<Pose2d> robotPoseSupplier, BooleanSupplier validity, DcMotorEx rightFront, DcMotorEx leftFront, DcMotorEx rightRear, DcMotorEx leftRear, BNO055IMU imu, axis rotationAxis, double trackWidth, double wheelRadius, double ticksPerRev, double xMultiplier, double yMultiplier, double xVariance, double yVariance, double headingVariance) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.validity = validity;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.rightRear = rightRear;
        this.leftRear = leftRear;
        this.imu = imu;
        this.rotationAxis = rotationAxis;
        this.trackWidth = trackWidth;
        this.wheelRadius = wheelRadius;
        this.ticksPerRev = ticksPerRev;
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.xVariance = xVariance;
        this.yVariance = yVariance;
        this.headingVariance = headingVariance;
        rightFrontPreviousPosition = rightFront.getCurrentPosition();
        leftFrontPreviousPosition = leftFront.getCurrentPosition();
        rightRearPreviousPosition = rightRear.getCurrentPosition();
        leftRearPreviousPosition = leftRear.getCurrentPosition();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void update() {

        double reading;

        switch (rotationAxis) {
            case X: reading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
                break;
            case Y: reading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
                break;
            default: reading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
                break;
        }

        double angleChange = reading - robotPoseSupplier.get().getHeading();

        int rightFrontPosition = rightFront.getCurrentPosition();
        double rightFrontDistance = (rightFrontPosition - rightFrontPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        double rightFrontTranslationDistance = rightFrontDistance - (angleChange * (2 * Math.PI * wheelRadius) * trackWidth * 2);
        rightFrontPreviousPosition = rightFrontPosition;

        int leftFrontPosition = leftFront.getCurrentPosition();
        double leftFrontDistance = (leftFrontPosition - leftFrontPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        double leftFrontTranslationDistance = leftFrontDistance - (angleChange * (2 * Math.PI * wheelRadius) * trackWidth * 2);
        leftFrontPreviousPosition = leftFrontPosition;

        int rightRearPosition = rightRear.getCurrentPosition();
        double rightRearDistance = (rightRearPosition - rightRearPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        double rightRearTranslationDistance = rightRearDistance - (angleChange * (2 * Math.PI * wheelRadius) * trackWidth * 2);
        rightRearPreviousPosition = rightRearPosition;

        int leftRearPosition = leftRear.getCurrentPosition();
        double leftRearDistance = (leftRearPosition - leftRearPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        double leftRearTranslationDistance = leftRearDistance - (angleChange * (2 * Math.PI * wheelRadius) * trackWidth * 2);
        leftRearPreviousPosition = leftRearPosition;


        //this code sets the 2 highest translation values to 0, so we can disregard them in the final calculations
        double absMaxDist = Math.max(Math.max(Math.max(Math.abs(rightFrontTranslationDistance), Math.abs(leftFrontTranslationDistance)), Math.abs(rightRearTranslationDistance)), Math.abs(leftRearTranslationDistance));

        if (Math.abs(rightFrontTranslationDistance) == absMaxDist) {rightFrontTranslationDistance = 0;}
        if (Math.abs(rightRearTranslationDistance) == absMaxDist) {rightRearTranslationDistance = 0;}
        if (Math.abs(leftFrontTranslationDistance) == absMaxDist) {leftFrontTranslationDistance = 0;}
        if (Math.abs(leftRearTranslationDistance) == absMaxDist) {leftRearTranslationDistance = 0;}

        absMaxDist = Math.max(Math.max(Math.max(Math.abs(rightFrontTranslationDistance), Math.abs(leftFrontTranslationDistance)), Math.abs(rightRearTranslationDistance)), Math.abs(leftRearTranslationDistance));

        if (Math.abs(rightFrontTranslationDistance) == absMaxDist) {rightFrontTranslationDistance = 0;}
        if (Math.abs(rightRearTranslationDistance) == absMaxDist) {rightRearTranslationDistance = 0;}
        if (Math.abs(leftFrontTranslationDistance) == absMaxDist) {leftFrontTranslationDistance = 0;}
        if (Math.abs(leftRearTranslationDistance) == absMaxDist) {leftRearTranslationDistance = 0;}

        localChange = new Pose2d(
                (rightFrontDistance + leftFrontDistance + rightRearDistance + leftRearDistance) * xMultiplier / 2,
                (-rightFrontDistance + leftFrontDistance + rightRearDistance - leftRearDistance) * yMultiplier / 2,
                new Rotation2d( angleChange)
        );
    }

    @Override
    public Pose2d getLocalPoseUpdate() {
        return localChange;
    }

    @Override
    public CovarianceMatrix getCovarianceUpdate() {
        return new CovarianceMatrix(
                xVariance * localChange.getX(),
                yVariance * localChange.getY(),
                0
        );
    }

    @Override
    public double getHeadingVarianceUpdate() {
        return headingVariance * localChange.getHeading();
    }

    @Override
    public boolean getValidity() {
        return validity.getAsBoolean();
    }

    @Override
    public boolean getCalculateIfInvalid() {
        return false;
    }
}
