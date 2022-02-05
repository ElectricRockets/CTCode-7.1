package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ScoreDuck;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.RedConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class DuckRed extends LinearOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode;
    private TrajectorySequence midBarcode;
    private TrajectorySequence rightBarcode;
    private TrajectorySequence scoreDuck;

    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(hardwareMap, telemetry);
        robot.initAuto(RedConstants.DUCK_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.DUCK_START)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .splineTo(RedConstants.DUCK_TSELEFT.vec(), RedConstants.DUCK_TSELEFT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_CLOSED))))
                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN))))
                .build();

        midBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.DUCK_START)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .splineTo(RedConstants.DUCK_TSEMID.vec(), RedConstants.DUCK_TSEMID.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_CLOSED))))
                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_OPEN))))
                .build();

        rightBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.DUCK_START)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .splineTo(RedConstants.DUCK_TSERIGHT.vec(), RedConstants.DUCK_TSERIGHT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .build();

        scoreDuck = robot.drive.trajectorySequenceBuilder(leftBarcode.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_TSE_HIGH))))
                .splineTo(RedConstants.CAROUSEL.vec(), RedConstants.CAROUSEL.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(RobotConstants.WAIT_BETWEEN_MOVEMENTS, () -> robot.schedule( new ScoreDuck(robot.duckSubsystem, ScoreDuck.fieldSides.RED, ScoreDuck.scoreTypes.OUTSIDE)))
                .waitSeconds(RobotConstants.UNIVERSAL_DELIVERY_TIME + RobotConstants.WAIT_BETWEEN_MOVEMENTS * 2)
                .splineTo(RedConstants.DUCKINTAKESTART.vec(), RedConstants.DUCKINTAKESTART.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                .lineToConstantHeading(RedConstants.DUCKINTAKEEND)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED_TSE_HIGH))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .setReversed(true)
                .splineTo(RedConstants.DUCK_DEPOSIT.vec(), RedConstants.DUCK_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.schedule(new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .setReversed(false)
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .splineTo( RedConstants.DEPOT_PARK.vec(), RedConstants.DEPOT_PARK.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        //waits until start and displays telemetry
        robot.waitForStart(this::isStarted);

        //chooses the correct trajectory based off of the randomization result
        TrajectorySequence firstSequence;
        switch (robot.cameraSubsystem.getResult()) {
            case LEFT : firstSequence = leftBarcode;
                break;
            case MID : firstSequence = midBarcode;
                break;
            default: firstSequence = rightBarcode;
                break;
        }

        //stops streaming to reduce computation usage.
        //robot.cameraSubsystem.stopStreaming();
        robot.cameraSubsystem.setState(CameraSubsystem.states.STORE);

        //schedules the trajectories for the full autonomous program in a sequential order
        robot.schedule(new SequentialCommandGroup(
                new TrajectorySequenceFollower( robot.drive, firstSequence),
                new TrajectorySequenceFollower(robot.drive, scoreDuck)
        ));

        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.update();
        }

        robot.reset();
    }
}
