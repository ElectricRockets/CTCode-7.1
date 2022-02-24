package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.constants.AutoTrajectories;
import org.firstinspires.ftc.teamcode.constants.BlueConstants;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class CycleBlue3 extends LinearOpMode {

    //creates an instance of the robot
    FFRobot robot;

    //sets up trajectories that will be generated in initialization
    private TrajectorySequence leftBarcode, midBarcode, rightBarcode, scoreFreight1, scoreFreight2, scoreFreight3, park;

    public void initialize() {
        //sets up the robot for autonomous
        robot = new FFRobot(this);
        robot.initAuto(BlueConstants.CYCLE_START);

        //generates all the trajectories that could be needed in order to reduce on-the-fly computation.
        leftBarcode = AutoTrajectories.startToHub(robot, BlueConstants.CYCLE_START, BlueConstants.CYCLE_TSELEFT, BlueConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_LOW_CLOSED);
        midBarcode = AutoTrajectories.startToHub(robot, BlueConstants.CYCLE_START, BlueConstants.CYCLE_TSEMID, BlueConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_MID_CLOSED);
        rightBarcode = AutoTrajectories.startToHub(robot, BlueConstants.CYCLE_START, BlueConstants.CYCLE_TSERIGHT, BlueConstants.CYCLE_DEPOSIT_REVERSED, LiftSubsystem.states.SCORE_HIGH_CLOSED);

        scoreFreight1 = AutoTrajectories.cycle(robot, leftBarcode.end(), BlueConstants.GAP, BlueConstants.GAP_INSIDE, BlueConstants.IW1, BlueConstants.IW2, BlueConstants.IW2_OFFSET,0,1, BlueConstants.CYCLE_DEPOSIT, BlueConstants.DEPOSIT_VARIANCE);
        scoreFreight2 = AutoTrajectories.cycle(robot, scoreFreight1.end(), BlueConstants.GAP, BlueConstants.GAP_INSIDE, BlueConstants.IW1, BlueConstants.IW2, BlueConstants.IW2_OFFSET,1,-1, BlueConstants.CYCLE_DEPOSIT, BlueConstants.DEPOSIT_VARIANCE);
        scoreFreight3 = AutoTrajectories.cycle(robot, scoreFreight2.end(), BlueConstants.GAP, BlueConstants.GAP_INSIDE, BlueConstants.IW1, BlueConstants.IW2, BlueConstants.IW2_OFFSET,2,1, BlueConstants.CYCLE_DEPOSIT, BlueConstants.DEPOSIT_VARIANCE);

        park = AutoTrajectories.warehousePark(robot, scoreFreight3.end(), BlueConstants.GAP, BlueConstants.GAP_INSIDE, BlueConstants.WAREHOUSE_PARK);
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
