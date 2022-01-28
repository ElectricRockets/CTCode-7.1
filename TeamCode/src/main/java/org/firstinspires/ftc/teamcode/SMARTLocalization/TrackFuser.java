package org.firstinspires.ftc.teamcode.SMARTLocalization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

public class TrackFuser implements Localizer {

    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d(0));
    private Pose2d priorRobotPose = robotPose;
    private int updateCount = 0;
    private double updateTime = System.nanoTime() * Math.pow(10,-9);
    private double priorUpdateTime = updateTime - 0.001;

    MecanumLocalizer mecanumLocalizer;
    DeadwheelLocalizer deadwheelLocalizer;

    DistanceSensorLocalizer distanceSensorLocalizer;

    RevIMU imu;
    double imuOffset;

    public TrackFuser(HardwareMap hardwareMap, BooleanSupplier mecanumValidity, BooleanSupplier deadwheelValidity) {
        mecanumLocalizer = new MecanumLocalizer(
                () -> LocalizationMath.toFtcLib(getPoseEstimate()),
                mecanumValidity,
                hardwareMap.get(DcMotorEx.class,"rightFront"),
                hardwareMap.get(DcMotorEx.class,"leftFront"),
                hardwareMap.get(DcMotorEx.class,"rightRear"),
                hardwareMap.get(DcMotorEx.class,"leftRear"),
                215.45,
                228,
                48,
                399
        );

        deadwheelLocalizer = new DeadwheelLocalizer(
                () -> LocalizationMath.toFtcLib(getPoseEstimate()),
                deadwheelValidity,
                hardwareMap.get(DcMotorEx.class, "rightEncoder"),
                hardwareMap.get(DcMotorEx.class, "leftEncoder"),
                hardwareMap.get(DcMotorEx.class, "intake"),
                206,
                -143,
                17.5,
                8192,
                1.0,
                1.0
        );

        distanceSensorLocalizer = new DistanceSensorLocalizer(() -> LocalizationMath.toFtcLib(getPoseEstimate()));

        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-97, -137, new Rotation2d(Math.toRadians(-90))), DistanceSensorEx.dSTypes.REV2M, true, hardwareMap.get(DistanceSensor.class, "dsL")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-97, 137, new Rotation2d(Math.toRadians(90))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-160, 64, new Rotation2d(Math.toRadians(180))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsRR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-160, -64, new Rotation2d(Math.toRadians(180))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsRL")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(206, 107, new Rotation2d(Math.toRadians(0))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsFR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(206, -107, new Rotation2d(Math.toRadians(0))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsFL")));

        imu = new RevIMU(hardwareMap, "IMU");
        imuOffset = 0;
    }

    @Override
    public void update() {
        updateCount++;
        //Pose2d currentVelocity = LocalizationMath.inToMM(LocalizationMath.toFtcLib(getPoseVelocity()));
        priorRobotPose = robotPose;
        priorUpdateTime = updateTime;
        updateTime = System.nanoTime() * Math.pow(10,-9);

        deadwheelLocalizer.update();
        mecanumLocalizer.update();

        //uses deadwheel if available, otherwise uses mecanum localization
        robotPose = deadwheelLocalizer.getValidity() ?
                LocalizationMath.addValues(robotPose, deadwheelLocalizer.lastPoseChange) :
                LocalizationMath.addValues(robotPose, mecanumLocalizer.lastPoseChange);

        //uses IMU every 20 loops, or every loop when using mecanum localization
        if( updateCount % 20 == 0 || !deadwheelLocalizer.getValidity()) {
            robotPose.rotate(robotPose.getHeading() + (imu.getHeading() + imuOffset));
        }

        //uses 1 distance sensor every 10 loops
        if( updateCount % 10 == 0) {
            distanceSensorLocalizer.update();
            robotPose = LocalizationMath.addValues(robotPose, distanceSensorLocalizer.getPoseCorrection());
        }
    }

    @Override
    @NonNull
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return LocalizationMath.toRR(LocalizationMath.mmToIn(robotPose));
    }

    @Override
    public void setPoseEstimate(@NonNull com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        robotPose = LocalizationMath.inToMM(LocalizationMath.toFtcLib(pose2d));
        priorRobotPose = robotPose;
        imuOffset = robotPose.getHeading();
        imu.reset();
    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return LocalizationMath.toRR(LocalizationMath.mmToIn( new Pose2d(
                (robotPose.getX() - priorRobotPose.getX()) / (updateTime - priorUpdateTime),
                (robotPose.getY() - priorRobotPose.getY()) / (updateTime - priorUpdateTime),
                new Rotation2d((robotPose.getHeading() - priorRobotPose.getHeading()) / (updateTime - priorUpdateTime)))));
    }

    public void setWallOffsets(double[] newWallOffsets) {
        distanceSensorLocalizer.setWallOffsets(newWallOffsets);
    }
}
