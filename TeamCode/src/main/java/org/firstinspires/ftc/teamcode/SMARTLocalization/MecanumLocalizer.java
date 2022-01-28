package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class MecanumLocalizer {

    Supplier<Pose2d> robotPoseSupplier;
    BooleanSupplier validity;
    DcMotorEx rightFront, leftFront, rightRear, leftRear;
    double trackWidth, wheelBase, wheelRadius, ticksPerRev;
    int rightFrontPreviousPosition, leftFrontPreviousPosition, rightRearPreviousPosition,leftRearPreviousPosition;
    Pose2d lastPoseChange;

    public MecanumLocalizer(Supplier<Pose2d> robotPoseSupplier, BooleanSupplier validity, DcMotorEx rightFront, DcMotorEx leftFront, DcMotorEx rightRear, DcMotorEx leftRear, double trackWidth, double wheelBase, double wheelRadius, double ticksPerRev) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.validity = validity;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.rightRear = rightRear;
        this.leftRear = leftRear;
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.wheelRadius = wheelRadius;
        this.ticksPerRev = ticksPerRev;
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

        Pose2d localChange = new Pose2d(
                (rightFrontDistance + leftFrontDistance + rightRearDistance + leftRearDistance)/4,
                (-rightFrontDistance + leftFrontDistance + rightRearDistance + leftRearDistance)/4,
                new Rotation2d((-rightFrontDistance + leftFrontDistance - rightRearDistance + leftRearDistance)/(4 * Math.sqrt(2) * Math.hypot(trackWidth/2, wheelBase/2)))
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
