package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.TrackFuser;
import org.firstinspires.ftc.teamcode.localization.trackestimators.DistanceSensorEstimator;
import org.firstinspires.ftc.teamcode.localization.trackupdater.MecanumUpdater;

public class FFRobotLocalizer {

    public static TrackFuser get(HardwareMap hardwareMap, Telemetry telemetry) {
        TrackFuser trackFuser = new TrackFuser(telemetry);

        trackFuser.setTrackUpdaters(
                new MecanumUpdater(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        () -> true,
                        hardwareMap.get(DcMotorEx.class,"rightFront"),
                        hardwareMap.get(DcMotorEx.class,"leftFront"),
                        hardwareMap.get(DcMotorEx.class,"rightRear"),
                        hardwareMap.get(DcMotorEx.class,"leftRear"),
                        hardwareMap.get(BNO055IMU.class, "imu"),
                        MecanumUpdater.axis.X,
                        8.48,
                        1.8897,
                        399,
                        1.0,
                        1.0,
                        1.0,
                        1.0,
                        trackFuser.getRobotHeadingVariance()
                )
        );

        trackFuser.setTrackEstimators(
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(-3.82,5.39, new Rotation2d(Math.toRadians(90))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsL"),
                        new double[] {0,0,0,0}
                ),
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(-3.82,-5.39, new Rotation2d(Math.toRadians(-90))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsR"),
                        new double[] {0,0,0,0}
                ),
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(8.11,4.21, new Rotation2d(Math.toRadians(0))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsFL"),
                        new double[] {0,0,0,0}
                ),
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(8.11,-4.21, new Rotation2d(Math.toRadians(0))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsFR"),
                        new double[] {0,0,0,0}
                ),
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(-6.30,2.52, new Rotation2d(Math.toRadians(180))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsRL"),
                        new double[] {0,0,0,0}
                ),
                new DistanceSensorEstimator(
                        telemetry,
                        trackFuser::getFTCLibPoseEstimate,
                        new Pose2d(-6.30,-2.52, new Rotation2d(Math.toRadians(180))),
                        DistanceSensorEstimator.DSTypes.REV2M,
                        true,
                        hardwareMap.get(DistanceSensor.class, "dsRR"),
                        new double[] {0,0,0,0}
                )
        );

        return trackFuser;
    }
}
