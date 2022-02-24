package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.subsystem.FFRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SimplePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FFRobot robot = new FFRobot(this);
        robot.initAuto(new Pose2d(0,0,0));

        TrajectorySequence park = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToConstantHeading(new Vector2d(20,0))
                .build();

        robot.waitForStartAuto();

        robot.schedule(new TrajectorySequenceFollower(robot.drive, park));

        robot.loop();

        robot.reset();
    }
}
