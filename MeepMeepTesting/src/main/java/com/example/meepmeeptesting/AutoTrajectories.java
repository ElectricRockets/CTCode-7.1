package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    /*
    public static TrajectorySequence cycle(/*FFRobot robot,*//* Pose2d startPose, Pose2d gapOutside, Pose2d gapInside, Pose2d IW1, Vector2d IW2, Vector2d collectOffset, double collectMultiplier, double depositOffsetMultiplier, Pose2d deposit, Vector2d depositOffset) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(RobotConstants.LEAVING_HUB_DIST_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(gapOutside.vec(), gapOutside.getHeading())
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                .splineTo(gapInside.vec(), gapInside.getHeading())
                .splineToSplineHeading(IW1, 0)
                .lineToConstantHeading(IW2.plus(collectOffset.times(collectMultiplier)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED))))
                .setTangent(toRadians(180))
                .lineToConstantHeading(IW1.vec())
                .splineToSplineHeading(gapInside, gapInside.getHeading() + toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .splineTo(gapOutside.vec(), gapOutside.getHeading() + toRadians(180))
                .splineTo(deposit.vec().plus(depositOffset.times(depositOffsetMultiplier)), deposit.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .build();
    }

    public static TrajectorySequence warehousePark(/*FFRobot robot,*//* Pose2d startPose, Pose2d gapOutside, Pose2d gapInside, Pose2d park) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                .UNSTABLE_addDisplacementMarkerOffset(RobotConstants.LEAVING_HUB_DIST_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(gapOutside.vec(), gapOutside.getHeading())
                .splineTo(gapInside.vec(), gapInside.getHeading())
                .splineTo(park.vec(), park.getHeading())
                .build();
    }*/
}
