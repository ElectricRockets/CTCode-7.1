package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoTrajectories {

    public static TrajectorySequence startToHub(FFRobot robot, Pose2d startPose, Pose2d grabTSEPose, Pose2d depositPoseReversed) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(grabTSEPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_CLOSED))))
                .lineToSplineHeading(depositPoseReversed)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN))))
                .build();
    }

    public static TrajectorySequence cycle(FFRobot robot, Pose2d startPose, Pose2d gapOutside, Pose2d gapInside, Pose2d IW1, Vector2d IW2, Vector2d collectOffset, double collectMultiplier, double depositOffsetMultiplier, Pose2d deposit, Vector2d depositOffset) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(RobotConstants.LEAVING_HUB_DIST_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(gapOutside.vec(), gapOutside.getHeading())
                .UNSTABLE_addDisplacementMarkerOffset(RedConstants.CYCLE_INTAKE_START_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
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

    public static TrajectorySequence warehousePark(FFRobot robot, Pose2d startPose, Pose2d gapOutside, Pose2d gapInside, Pose2d park) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                .UNSTABLE_addDisplacementMarkerOffset(RobotConstants.LEAVING_HUB_DIST_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(gapOutside.vec(), gapOutside.getHeading())
                .splineTo(gapInside.vec(), gapInside.getHeading())
                .splineTo(park.vec(), park.getHeading())
                .build();
    }
}
