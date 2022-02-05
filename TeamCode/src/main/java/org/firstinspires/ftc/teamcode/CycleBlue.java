package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.BlueConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class CycleBlue extends CommandOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode;
    private TrajectorySequence midBarcode;
    private TrajectorySequence rightBarcode;
    private TrajectorySequence scoreFreight;
    private TrajectorySequence park;


    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(hardwareMap, telemetry);
        robot.initAuto(BlueConstants.CYCLE_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = robot.drive.trajectorySequenceBuilder(BlueConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_TSELEFT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN))))
                .build();

        midBarcode = robot.drive.trajectorySequenceBuilder(BlueConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_TSEMID)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_OPEN))))
                .build();

        rightBarcode = robot.drive.trajectorySequenceBuilder(BlueConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_TSERIGHT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .lineToSplineHeading(BlueConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .build();

        scoreFreight = robot.drive.trajectorySequenceBuilder(leftBarcode.end())
                .UNSTABLE_addTemporalMarkerOffset(BlueConstants.LEAVING_HUB_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading())
                .UNSTABLE_addDisplacementMarkerOffset(BlueConstants.CYCLE_INTAKE_START_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                .splineTo(BlueConstants.CYCLE_COLLECT_INITIAL.vec(), BlueConstants.CYCLE_COLLECT_INITIAL.getHeading())
                .setReversed(true)
                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading() + Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(BlueConstants.CYCLE_LIFT_EXTEND_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .splineTo(BlueConstants.CYCLE_DEPOSIT.vec(), BlueConstants.CYCLE_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .build();

        park = robot.drive.trajectorySequenceBuilder(scoreFreight.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(BlueConstants.LEAVING_HUB_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(BlueConstants.GAP.vec(), BlueConstants.GAP.getHeading())
                .splineTo(BlueConstants.WAREHOUSE_PARK.vec(), BlueConstants.WAREHOUSE_PARK.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException{

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
                new TrajectorySequenceFollower(robot.drive, park)
        ));

        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.update();
        }

        robot.reset();
    }
}
