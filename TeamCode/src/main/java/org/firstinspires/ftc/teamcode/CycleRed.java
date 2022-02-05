package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.AutoTrajectories;
import org.firstinspires.ftc.teamcode.constants.RedConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class CycleRed extends CommandOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode;
    private TrajectorySequence midBarcode;
    private TrajectorySequence rightBarcode;
    private TrajectorySequence scoreFreight1;
    private TrajectorySequence scoreFreight2;
    private TrajectorySequence scoreFreight3;
    private TrajectorySequence scoreFreight4;
    private TrajectorySequence park;


    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(hardwareMap, telemetry);
        robot.initAuto(RedConstants.CYCLE_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSELEFT, RedConstants.CYCLE_DEPOSIT_REVERSED);
        midBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSEMID, RedConstants.CYCLE_DEPOSIT_REVERSED);
        rightBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSERIGHT, RedConstants.CYCLE_DEPOSIT_REVERSED);

        scoreFreight1 = AutoTrajectories.cycle(robot, leftBarcode.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, RedConstants.IW2_OFFSET,0,1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);
        scoreFreight2 = AutoTrajectories.cycle(robot, scoreFreight1.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, RedConstants.IW2_OFFSET,1,-1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);
        scoreFreight3 = AutoTrajectories.cycle(robot, scoreFreight2.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, RedConstants.IW2_OFFSET,2,1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);
        scoreFreight4 = AutoTrajectories.cycle(robot, scoreFreight3.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, RedConstants.IW2_OFFSET,3,-1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);

        park = AutoTrajectories.warehousePark(robot, scoreFreight4.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.WAREHOUSE_PARK);

        /*
        leftBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_TSELEFT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_LOW_OPEN))))
                .build();

        midBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_TSEMID)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_MID_OPEN))))
                .build();

        rightBarcode = robot.drive.trajectorySequenceBuilder(RedConstants.CYCLE_START)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_OPEN_INTAKE_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_TSERIGHT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.GRAB_TSE_CLOSED_INTAKE_CLOSED))))
                .waitSeconds(RobotConstants.WAIT_BETWEEN_MOVEMENTS)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule( new InstantCommand( () -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .lineToSplineHeading(RedConstants.CYCLE_DEPOSIT_REVERSED)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .build();
        */

        /*
        scoreFreight1 = scoreFreight(1, 0, leftBarcode.end());
        scoreFreight2 = scoreFreight(-1, 1, scoreFreight1.end());
        scoreFreight3 = scoreFreight(1, 2, scoreFreight2.end());
        scoreFreight4 = scoreFreight(-1, 3, scoreFreight3.end());

        park = robot.drive.trajectorySequenceBuilder(scoreFreight4.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.RETRACTED))))
                .UNSTABLE_addTemporalMarkerOffset(RedConstants.LEAVING_HUB_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                .splineTo(RedConstants.WAREHOUSE_PARK.vec(), RedConstants.WAREHOUSE_PARK.getHeading())
                .build();
        */
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
                new TrajectorySequenceFollower(robot.drive, scoreFreight1),
                new TrajectorySequenceFollower(robot.drive, scoreFreight2),
                new TrajectorySequenceFollower(robot.drive, scoreFreight3),
                new TrajectorySequenceFollower(robot.drive, scoreFreight4),
                new TrajectorySequenceFollower(robot.drive, park)
        ));

        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.update();
        }

        robot.reset();
    }

    private TrajectorySequence scoreFreight(double hubMultiplier, double collectMultiplier, Pose2d startPose) {
        return robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(RedConstants.LEAVING_HUB_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE))))
                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading())
                .UNSTABLE_addDisplacementMarkerOffset(RedConstants.CYCLE_INTAKE_START_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.intakeSubsystem.setState(IntakeSubsystem.states.SMART_INTAKE))))
                .splineTo(RedConstants.GAP_INSIDE.vec(), RedConstants.GAP_INSIDE.getHeading())
                .splineToSplineHeading(RedConstants.IW1, 0)
                .lineToConstantHeading(RedConstants.IW2.plus(RedConstants.IW2_OFFSET.times(collectMultiplier)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.INTAKE_CLOSED))))
                .setTangent(toRadians(180))
                .lineToConstantHeading(RedConstants.IW1.vec())
                .splineToSplineHeading(RedConstants.GAP_INSIDE, RedConstants.GAP_INSIDE.getHeading() + toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(RedConstants.CYCLE_LIFT_EXTEND_OFFSET, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_CLOSED))))
                .splineTo(RedConstants.GAP.vec(), RedConstants.GAP.getHeading() + toRadians(180))
                .splineTo(RedConstants.CYCLE_DEPOSIT.vec().plus(RedConstants.DEPOSIT_VARIANCE.times(hubMultiplier)), RedConstants.CYCLE_DEPOSIT.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.schedule(new InstantCommand(() -> robot.liftSubsystem.setState(LiftSubsystem.states.SCORE_HIGH_OPEN))))
                .build();
    }
}
