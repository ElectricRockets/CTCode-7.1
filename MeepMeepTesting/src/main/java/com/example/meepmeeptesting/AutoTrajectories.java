package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.jetbrains.annotations.NotNull;

import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class AutoTrajectories {

    public static TrajectorySequence startToHub(/*FFRobot robot,*/ Pose2d startPose, Pose2d grabTSEPose, Pose2d depositPoseReversed) {
        return new TrajectorySequenceBuilder(BlueConstants.CYCLE_START, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 50;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 50;
            }
        }, toRadians(360),
                toRadians(360))
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(grabTSEPose)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_CLOSED))))
                .lineToSplineHeading(depositPoseReversed)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN))))
                .build();
    }
    public static TrajectorySequence scoreDuck(/*FFRobot robot,*/ Pose2d startPose, Pose2d carousel, /*ScoreDuck.fieldSides side,*/ Pose2d duckIntakeStart, Vector2d duckIntakeEnd, Pose2d duckDeposit, Pose2d depotPark) {
        return new TrajectorySequenceBuilder(startPose, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 50;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 50;
            }
        }, toRadians(360),
                toRadians(360))
                //.UNSTABLE_addDisplacementMarkerOffset(
                //        RobotConstants.LEAVING_HUB_DIST_OFFSET,
                //        () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_TSE_HIGH)))
                //)
                .lineToSplineHeading(carousel)
                //.UNSTABLE_addTemporalMarkerOffset(
                //        RobotConstants.WAIT_BETWEEN_MOVEMENTS,
                //        () -> robot.schedule( new ScoreDuck(robot.duckSubsystem, side, ScoreDuck.scoreTypes.OUTSIDE))
                //)
                .waitSeconds(RobotConstants.UNIVERSAL_DELIVERY_TIME + RobotConstants.WAIT_BETWEEN_MOVEMENTS * 2)
                .setTangent(toRadians(45))
                .splineTo(duckIntakeStart.vec(), duckIntakeStart.getHeading())
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0,
                //        () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE)))
                //)
                .lineToConstantHeading(duckIntakeEnd)
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0,
                //        () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED_TSE_HIGH)))
                //)
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0,
                //        () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED)))
                //)
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0.3,
                //        () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED)))
                //)
                .setReversed(true)
                .splineTo(duckDeposit.vec(), duckDeposit.getHeading())
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0.3,
                //        () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN)))
                //)
                .setReversed(false)
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                //.UNSTABLE_addTemporalMarkerOffset(
                //        0,
                //        () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE)))
                //)
                .splineTo( depotPark.vec(), depotPark.getHeading())
                .build();
    }
}
