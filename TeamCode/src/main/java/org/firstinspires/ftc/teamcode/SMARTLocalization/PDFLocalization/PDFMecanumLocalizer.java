package org.firstinspires.ftc.teamcode.SMARTLocalization.PDFLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PDFMecanumLocalizer {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<CovarianceMatrix> robotXYCovarianceSupplier;
    private final DoubleSupplier robotHeadingVarianceSupplier;
    private final BooleanSupplier validity;
    private final DcMotorEx rightFront, leftFront, rightRear, leftRear;
    private final double trackWidth, wheelBase, wheelRadius, ticksPerRev, xMultiplier, yMultiplier, angleMultiplier;
    private int rightFrontPreviousPosition, leftFrontPreviousPosition, rightRearPreviousPosition,leftRearPreviousPosition;
    private Pose2d poseChange;
    private Pose2d localChange;

    double xVariance;
    double yVariance;
    double headingVariance;

    public PDFMecanumLocalizer(Supplier<Pose2d> robotPoseSupplier, Supplier<CovarianceMatrix> robotXYCovarianceSupplier, DoubleSupplier robotHeadingVarianceSupplier, BooleanSupplier validity, DcMotorEx rightFront, DcMotorEx leftFront, DcMotorEx rightRear, DcMotorEx leftRear, double trackWidth, double wheelBase, double wheelRadius, double ticksPerRev, double xMultiplier, double yMultiplier, double angleMultiplier, double xVariance, double yVariance, double headingVariance) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotXYCovarianceSupplier = robotXYCovarianceSupplier;
        this.robotHeadingVarianceSupplier = robotHeadingVarianceSupplier;
        this.validity = validity;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.rightRear = rightRear;
        this.leftRear = leftRear;
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.wheelRadius = wheelRadius;
        this.ticksPerRev = ticksPerRev;
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.angleMultiplier = angleMultiplier;
        this.xVariance = xVariance;
        this.yVariance = yVariance;
        this.headingVariance = headingVariance;
        rightFrontPreviousPosition = rightFront.getCurrentPosition();
        leftFrontPreviousPosition = leftFront.getCurrentPosition();
        rightRearPreviousPosition = rightRear.getCurrentPosition();
        leftRearPreviousPosition = leftRear.getCurrentPosition();
    }

    public void update() {
        int rightFrontPosition = rightFront.getCurrentPosition();
        double rightFrontDistance = (rightFrontPosition - rightFrontPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        rightFrontPreviousPosition = rightFrontPosition;

        int leftFrontPosition = leftFront.getCurrentPosition();
        double leftFrontDistance = (leftFrontPosition - leftFrontPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        leftFrontPreviousPosition = leftFrontPosition;

        int rightRearPosition = rightRear.getCurrentPosition();
        double rightRearDistance = (rightRearPosition - rightRearPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        rightRearPreviousPosition = rightRearPosition;

        int leftRearPosition = leftRear.getCurrentPosition();
        double leftRearDistance = (leftRearPosition - leftRearPreviousPosition) * (2 * Math.PI * wheelRadius) / ticksPerRev;
        leftRearPreviousPosition = leftRearPosition;

        localChange = new Pose2d(
                (rightFrontDistance + leftFrontDistance + rightRearDistance + leftRearDistance) * xMultiplier / 4,
                (-rightFrontDistance + leftFrontDistance + rightRearDistance + leftRearDistance) * yMultiplier / 4,
                new Rotation2d(angleMultiplier * (-rightFrontDistance + leftFrontDistance - rightRearDistance + leftRearDistance) / (4 * Math.sqrt(2) * Math.hypot(trackWidth/2, wheelBase/2)))
        );

        poseChange = LocalizationMath.localToGlobalPoseChange(localChange, robotPoseSupplier.get().getHeading());
    }

    public Pose2d getPoseChange() {
        return poseChange;
    }

    public CovarianceMatrix getLastCovariance() {
        return new CovarianceMatrix(
                xVariance * localChange.getX(),
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
