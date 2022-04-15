package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.AutoTrajectories;
import org.firstinspires.ftc.teamcode.constants.RedConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class CycleRed3 extends LinearOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode, midBarcode, rightBarcode, scoreFreight1, scoreFreight2, scoreFreight3, park;

    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(this);
        robot.initAuto(RedConstants.CYCLE_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSELEFT, RedConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_LOW_CLOSED);
        midBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSEMID, RedConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_MID_CLOSED);
        rightBarcode = AutoTrajectories.startToHub(robot, RedConstants.CYCLE_START, RedConstants.CYCLE_TSERIGHT, RedConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_HIGH_CLOSED);

        scoreFreight1 = AutoTrajectories.cycle(robot, leftBarcode.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, new Pose2d(0,0,0),0,1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);
        scoreFreight2 = AutoTrajectories.cycle(robot, scoreFreight1.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, new Pose2d(0,6,0),1,-1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);
        scoreFreight3 = AutoTrajectories.cycle(robot, scoreFreight2.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.IW1, RedConstants.IW2, new Pose2d(4,3,0),1,1, RedConstants.CYCLE_DEPOSIT, RedConstants.DEPOSIT_VARIANCE);

        park = AutoTrajectories.warehousePark(robot, scoreFreight2.end(), RedConstants.GAP, RedConstants.GAP_INSIDE, RedConstants.WAREHOUSE_PARK);
    }

    @Override
    public void runOpMode() throws InterruptedException{

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

        //stores the camera for safety and stops streaming
        robot.cameraSubsystem.setState(CameraSubsystem.states.STORE);

        //schedules the trajectories for the full autonomous program in a sequential order
        robot.schedule(new SequentialCommandGroup(
                new TrajectorySequenceFollower( robot.drive, firstSequence),
                new TrajectorySequenceFollower(robot.drive, scoreFreight1),
                new TrajectorySequenceFollower(robot.drive, scoreFreight2),
                new TrajectorySequenceFollower(robot.drive, scoreFreight3),
                new TrajectorySequenceFollower(robot.drive, park)
        ));

        robot.loop();

        robot.reset();
    }
}
