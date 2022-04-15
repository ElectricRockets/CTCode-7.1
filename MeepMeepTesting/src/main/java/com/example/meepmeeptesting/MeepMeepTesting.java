package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import org.jetbrains.annotations.NotNull;

import java.io.IOException;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity duckBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(25,25, Math.toRadians(180), Math.toRadians(180), 12.6)
                .setDimensions(11,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BlueConstants.DUCK_START)
                                .setReversed(true)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                                .splineTo(BlueConstants.DUCK_TSERIGHT.vec(), BlueConstants.DUCK_TSERIGHT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .splineTo(BlueConstants.DUCK_DEPOSIT.vec(), BlueConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .splineTo(BlueConstants.CAROUSEL.vec(), BlueConstants.CAROUSEL.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(RobotConstants.WAIT_BETWEEN_MOVEMENTS, () -> robot.schedule( new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.BLUE, () -> ScoreDuck.scoreTypes.UNIVERSAL)))
                                .waitSeconds(RobotConstants.UNIVERSAL_DELIVERY_TIME + RobotConstants.WAIT_BETWEEN_MOVEMENTS * 2)
                                .splineTo(BlueConstants.DUCKINTAKESTART.vec(), BlueConstants.DUCKINTAKESTART.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                                .lineToConstantHeading(BlueConstants.DUCKINTAKEEND)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .setReversed(true)
                                .splineTo(BlueConstants.DUCK_DEPOSIT.vec(), BlueConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                                .setReversed(false)
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                .splineTo( BlueConstants.DEPOT_PARK.vec(), BlueConstants.DEPOT_PARK.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .build()
                );

        RoadRunnerBotEntity duckBlue2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(25,25, Math.toRadians(180), Math.toRadians(180), 12.6)
                .setDimensions(11,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BlueConstants.DUCK_START)
                                .setReversed(true)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                                .lineToConstantHeading(BlueConstants.DUCK_TSELEFT.vec())
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .splineTo(BlueConstants.DUCK_DEPOSIT.vec(), BlueConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .splineTo(BlueConstants.CAROUSEL.vec(), BlueConstants.CAROUSEL.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(RobotConstants.WAIT_BETWEEN_MOVEMENTS, () -> robot.schedule( new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.BLUE, () -> ScoreDuck.scoreTypes.UNIVERSAL)))
                                .waitSeconds(RobotConstants.UNIVERSAL_DELIVERY_TIME + RobotConstants.WAIT_BETWEEN_MOVEMENTS * 2)
                                .splineTo(BlueConstants.DUCKINTAKESTART.vec(), BlueConstants.DUCKINTAKESTART.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                                .lineToConstantHeading(BlueConstants.DUCKINTAKEEND)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .setReversed(true)
                                .splineTo(BlueConstants.DUCK_DEPOSIT.vec(), BlueConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                                .setReversed(false)
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                .splineTo( BlueConstants.DEPOT_PARK.vec(), BlueConstants.DEPOT_PARK.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .build()
                );

        RoadRunnerBotEntity cycleBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(25,25, Math.toRadians(180), Math.toRadians(180), 12.6)
                .setDimensions(11,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence( AutoTrajectories.startToHub(/*robot,*/ BlueConstants.CYCLE_START, BlueConstants.CYCLE_TSELEFT, BlueConstants.CYCLE_DEPOSIT_REVERSED)
                );

        RoadRunnerBotEntity duckRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(25,25, Math.toRadians(180), Math.toRadians(180), 12.6)
                .setDimensions(11,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(AutoTrajectories.scoreDuck(/*robot,*/ RedConstants.DUCK_DEPOSIT_REVERSED, RedConstants.CAROUSEL, /*ScoreDuck.fieldSides.RED,*/ RedConstants.DUCKINTAKESTART, RedConstants.DUCKINTAKEEND, RedConstants.DUCK_DEPOSIT, RedConstants.DEPOT_PARK)
                );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(duckBlue2)
                .addEntity(cycleBlue)
                .addEntity(duckRed)
                .start();
    }
}