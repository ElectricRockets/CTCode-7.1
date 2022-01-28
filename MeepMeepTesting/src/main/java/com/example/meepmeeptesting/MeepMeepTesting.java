package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

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
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(BlueConstants.CYCLE_START)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                                .lineToSplineHeading(BlueConstants.CYCLE_TSERIGHT)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_LOW_CLOSED))))
                                .turn(BlueConstants.CYCLE_DEPOSIT_REVERSED.getHeading() - BlueConstants.CYCLE_TSERIGHT.getHeading() - toRadians(360))
                                .lineToSplineHeading(BlueConstants.CYCLE_DEPOSIT_REVERSED)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_LOW_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.INTAKE))))
                                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading())
                                //.UNSTABLE_addDisplacementMarkerOffset(6, () -> robot.schedule(new InstantCommand(robot.intakeSubsystem::smartIntake)))
                                .splineTo(BlueConstants.CYCLE_COLLECT.vec(), BlueConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading() + Math.toRadians(180))
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_HIGH_CLOSED))))
                                .splineTo(BlueConstants.CYCLE_DEPOSIT.vec(), BlueConstants.CYCLE_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_HIGH_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.INTAKE))))
                                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading())
                                //.UNSTABLE_addDisplacementMarkerOffset(6, () -> robot.schedule(new InstantCommand(robot.intakeSubsystem::smartIntake)))
                                .splineTo(BlueConstants.CYCLE_COLLECT.vec(), BlueConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading() + Math.toRadians(180))
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_HIGH_CLOSED))))
                                .splineTo(BlueConstants.CYCLE_DEPOSIT.vec(), BlueConstants.CYCLE_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.SCORE_HIGH_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setLiftState(LiftSubsystem.liftStates.INTAKE))))
                                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading())
                                .splineTo(BlueConstants.WAREHOUSE_PARK.vec(), BlueConstants.WAREHOUSE_PARK.getHeading())
                                .build()
                );

        RoadRunnerBotEntity duckRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(25,25, Math.toRadians(180), Math.toRadians(180), 12.6)
                .setDimensions(11,12.75)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RedConstants.DUCK_START)
                                .setReversed(true)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                                .splineTo(RedConstants.DUCK_TSERIGHT.vec(), RedConstants.DUCK_TSERIGHT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))

                                .setReversed(false)
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .splineTo(RedConstants.CAROUSEL.vec(), RedConstants.CAROUSEL.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(RobotConstants.WAIT_BETWEEN_MOVEMENTS, () -> robot.schedule( new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.Red, () -> ScoreDuck.scoreTypes.UNIVERSAL)))
                                .waitSeconds(RobotConstants.UNIVERSAL_DELIVERY_TIME + RobotConstants.WAIT_BETWEEN_MOVEMENTS * 2)
                                .splineTo(RedConstants.DUCKINTAKESTART.vec(), RedConstants.DUCKINTAKESTART.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                                .lineToConstantHeading(RedConstants.DUCKINTAKEEND)
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                                .setReversed(true)
                                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                                .setReversed(false)
                                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                                .splineTo( RedConstants.DEPOT_PARK.vec(), RedConstants.DEPOT_PARK.getHeading())
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                                .build()
                );

        RoadRunnerBotEntity cycleRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(65,150, Math.toRadians(360), Math.toRadians(360), 8.48)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(RedConstants.CYCLE_START)

                                .setReversed(true)
                                .splineTo( RedConstants.CYCLE_TSELEFT.vec(), RedConstants.CYCLE_TSELEFT.getHeading()) // Leftmost Barcode
                                .waitSeconds(0.2)
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.CYCLE_COLLECT.vec(), RedConstants.CYCLE_COLLECT.getHeading())
                                .setReversed(true)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                                .splineTo(RedConstants.CYCLE_DEPOSIT.vec(), RedConstants.CYCLE_DEPOSIT.getHeading())

                                .setReversed(false)
                                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                                .splineTo(RedConstants.WAREHOUSE_PARK.vec(), RedConstants.WAREHOUSE_PARK.getHeading())
                                .build()
                );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(duckBlue2)
                .addEntity(cycleBlue)
                .addEntity(duckRed)
                //.addEntity(cycleRed)
                .start();
    }
}