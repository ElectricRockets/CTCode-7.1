package org.firstinspires.ftc.teamcode.SMARTLocalization.PDFLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PDFDeadwheelLocalizer {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<CovarianceMatrix> robotXYCovarianceSupplier;
    private final DoubleSupplier robotHeadingVarianceSupplier;
    private final BooleanSupplier validity;
    private final DcMotorEx rightEncoder, leftEncoder, perpEncoder;
    private final double trackWidth, perpOffset, wheelRadius, ticksPerRev, xMultiplier, yMultiplier;
    private int rightPreviousPosition, leftPreviousPosition, perpPreviousPosition;
    private Pose2d poseChange;
    private Pose2d localChange;

    private final double xVariance;
    private final double yVariance;
    private final double headingVariance;

    public PDFDeadwheelLocalizer(Supplier<Pose2d> robotPoseSupplier, Supplier<CovarianceMatrix> robotXYCovarianceSupplier, DoubleSupplier robotHeadingVarianceSupplier, BooleanSupplier validity, DcMotorEx rightEncoder, DcMotorEx leftEncoder, DcMotorEx perpEncoder, double trackWidth, double perpOffset, double wheelRadius, double ticksPerRev, double xMultiplier, double yMultiplier, double xVariance, double yVariance, double headingVariance) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotXYCovarianceSupplier = robotXYCovarianceSupplier;
        this.robotHeadingVarianceSupplier = robotHeadingVarianceSupplier;
        this.validity = validity;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.perpEncoder = perpEncoder;
        this.trackWidth = trackWidth;
        this.perpOffset = perpOffset;
        this.wheelRadius = wheelRadius;
        this.ticksPerRev = ticksPerRev;
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.xVariance = xVariance;
        this.yVariance = yVariance;
        this.headingVariance = headingVariance;
        rightPreviousPosition = rightEncoder.getCurrentPosition();
        leftPreviousPosition = leftEncoder.getCurrentPosition();
        perpPreviousPosition = perpEncoder.getCurrentPosition();
    }

    public void update() {
        int rightPosition = rightEncoder.getCurrentPosition();
        double rightDistance = (rightPosition - rightPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        rightPreviousPosition = rightPosition;

        int leftPosition = leftEncoder.getCurrentPosition();
        double leftDistance = (leftPosition - leftPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        leftPreviousPosition = leftPosition;

        int perpPosition = perpEncoder.getCurrentPosition();
        double perpDistance = (perpPosition - perpPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        perpPreviousPosition = perpPosition;

        localChange = new Pose2d(
                (rightDistance + leftDistance) * xMultiplier / 2,
                (perpDistance + (((-rightDistance + leftDistance) / trackWidth) * perpOffset)) * yMultiplier,
                new Rotation2d((-rightDistance + leftDistance) / trackWidth)
        );

        poseChange = LocalizationMath.localToGlobalPoseChange(localChange, robotPoseSupplier.get().getHeading());
    }

    public Pose2d getPoseChange() {
        return poseChange;
    }

    public CovarianceMatrix getLastCovariance() {
        return new CovarianceMatrix(
                xVariance * localChange.getX() * robotHeadingVarianceSupplier.getAsDouble(),
                yVariance * localChange.getY(),
                robotPoseSupplier.get().getHeading());
    }

    public double getLastHeadingVariance() {
        return headingVariance * localChange.getHeading();
    }

    public boolean getValidity() {
        return validity.getAsBoolean();
    }
}
