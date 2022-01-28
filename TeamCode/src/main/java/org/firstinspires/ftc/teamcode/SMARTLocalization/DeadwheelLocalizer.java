package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DeadwheelLocalizer {

    Supplier<Pose2d> robotPoseSupplier;
    BooleanSupplier validity;
    DcMotorEx rightEncoder, leftEncoder, perpEncoder;
    double trackWidth, perpOffset, wheelRadius, ticksPerRev, xMultiplier, yMultiplier;
    int rightPreviousPosition, leftPreviousPosition, perpPreviousPosition;
    Pose2d lastPoseChange;

    public DeadwheelLocalizer(Supplier<Pose2d> robotPoseSupplier, BooleanSupplier validity, DcMotorEx rightEncoder, DcMotorEx leftEncoder, DcMotorEx perpEncoder, double trackWidth, double perpOffset, double wheelRadius, double ticksPerRev, double xMultiplier, double yMultiplier) {
        this.robotPoseSupplier = robotPoseSupplier;
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

        Pose2d localChange = new Pose2d(
                (rightDistance + leftDistance) * xMultiplier / 2,
                perpDistance + (((-rightDistance + leftDistance) / trackWidth) * perpOffset),
                new Rotation2d((-rightDistance + leftDistance) / trackWidth)
        );

        lastPoseChange = LocalizationMath.localToGlobalPoseChange(localChange, robotPoseSupplier.get().getHeading());
    }

    public Pose2d getLastPoseChange() {
        return lastPoseChange;
    }

    public boolean getValidity() {
        return validity.getAsBoolean();
    }
}
