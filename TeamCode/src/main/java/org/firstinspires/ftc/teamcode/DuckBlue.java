package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ScoreDuck;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.AutoTrajectories;
import org.firstinspires.ftc.teamcode.constants.BlueConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class DuckBlue extends LinearOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode, midBarcode, rightBarcode, scoreDuck;

    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(this);
        robot.initAuto(BlueConstants.DUCK_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = AutoTrajectories.startToHub(robot, BlueConstants.DUCK_START, BlueConstants.DUCK_TSELEFT, BlueConstants.DUCK_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_LOW_CLOSED);
        midBarcode = AutoTrajectories.startToHub(robot, BlueConstants.DUCK_START, BlueConstants.DUCK_TSEMID, BlueConstants.DUCK_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_MID_CLOSED);
        rightBarcode = AutoTrajectories.startToHub(robot, BlueConstants.DUCK_START, BlueConstants.DUCK_TSERIGHT, BlueConstants.DUCK_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_HIGH_CLOSED);

        scoreDuck = AutoTrajectories.scoreDuck(robot, leftBarcode.end(), BlueConstants.CAROUSEL, ScoreDuck.fieldSides.BLUE, BlueConstants.DUCKINTAKESTART, BlueConstants.DUCKINTAKEEND, BlueConstants.DUCK_DEPOSIT, BlueConstants.DEPOT_PARK);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        //waits until start and displays telemetry
        robot.waitForStartAuto();

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

        robot.loop();

        robot.reset();
    }
}
