package org.firstinspires.ftc.teamcode.SMARTLocalization.PDFLocalization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.DistanceSensorEx;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;

import java.util.function.BooleanSupplier;

public class PDFTrackFuser implements Localizer {

    Pose2d robotPoseEstimate = new Pose2d(0,0, new Rotation2d(0));
    CovarianceMatrix robotXYCovariance = new CovarianceMatrix(25,25,0); //reasonable XY Variance of robot placement
    double robotHeadingVariance = 0.035; //reasonable heading variance of robot placement

    private Pose2d priorRobotPoseEstimate = robotPoseEstimate;
    private int updateCount = -1; //is -1 that way when the first update happens, all functions that execute on different intervals execute
    private double updateTime = System.nanoTime() * Math.pow(10,-9);
    private double priorUpdateTime = updateTime - 0.001;

    private final double integrationErrorConstant = 0.1;


    PDFMecanumLocalizer mecanumLocalizer;
    PDFDeadwheelLocalizer deadwheelLocalizer;
    PDFDistanceSensorLocalizer distanceSensorLocalizer;
    PDFImu imu;

    public PDFTrackFuser(HardwareMap hardwareMap, BooleanSupplier deadwheelValidity, BooleanSupplier mecanumValidity) {
        mecanumLocalizer = new PDFMecanumLocalizer(
                () -> LocalizationMath.toFtcLib(getPoseEstimate()),
                this::getXYCovariance,
                this::getHeadingVariance,
                mecanumValidity,
                hardwareMap.get(DcMotorEx.class,"rightFront"),
                hardwareMap.get(DcMotorEx.class,"leftFront"),
                hardwareMap.get(DcMotorEx.class,"rightRear"),
                hardwareMap.get(DcMotorEx.class,"leftRear"),
                215.45,
                228,
                48,
                399,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0
        );

        deadwheelLocalizer = new PDFDeadwheelLocalizer(
                () -> LocalizationMath.toFtcLib(getPoseEstimate()),
                this::getXYCovariance,
                this::getHeadingVariance,
                deadwheelValidity,
                hardwareMap.get(DcMotorEx.class, "rightEncoder"),
                hardwareMap.get(DcMotorEx.class, "leftEncoder"),
                hardwareMap.get(DcMotorEx.class, "intake"),
                206,
                -143,
                17.5,
                8192,
                1.0,
                1.0,
                0.5,
                0.5,
                0.5
        );

        distanceSensorLocalizer = new PDFDistanceSensorLocalizer(
                () -> LocalizationMath.toFtcLib(getPoseEstimate()),
                this::getXYCovariance,
                this::getErrorVelocity
        );

        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-97, -137, new Rotation2d(Math.toRadians(-90))), DistanceSensorEx.dSTypes.REV2M, true, hardwareMap.get(DistanceSensor.class, "dsL")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-97, 137, new Rotation2d(Math.toRadians(90))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-160, 64, new Rotation2d(Math.toRadians(180))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsRR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(-160, -64, new Rotation2d(Math.toRadians(180))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsRL")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(206, 107, new Rotation2d(Math.toRadians(0))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsFR")));
        distanceSensorLocalizer.add(new DistanceSensorEx( new Pose2d(206, -107, new Rotation2d(Math.toRadians(0))), DistanceSensorEx.dSTypes.REV2M,true, hardwareMap.get(DistanceSensor.class, "dsFL")));

        imu = new PDFImu(hardwareMap.get(BNO055IMU.class, "IMU"),0,  0.1, 0.1, PDFImu.axis.X);
    }

    @Override
    public void update() {
        updateCount++;
        //Pose2d currentVelocity = LocalizationMath.inToMM(LocalizationMath.toFtcLib(getPoseVelocity()));
        priorRobotPoseEstimate = robotPoseEstimate;
        priorUpdateTime = updateTime;
        updateTime = System.nanoTime() * Math.pow(10,-9);

        deadwheelLocalizer.update();
        mecanumLocalizer.update();

        if (deadwheelLocalizer.getValidity()) {
            robotPoseEstimate = LocalizationMath.addValues(robotPoseEstimate, deadwheelLocalizer.getPoseChange());
            robotXYCovariance = robotXYCovariance.add(deadwheelLocalizer.getLastCovariance());
            robotHeadingVariance = robotHeadingVariance + deadwheelLocalizer.getLastHeadingVariance();
        } else {
            robotPoseEstimate = LocalizationMath.addValues(robotPoseEstimate, mecanumLocalizer.getPoseChange());
            robotXYCovariance = robotXYCovariance.add(mecanumLocalizer.getLastCovariance());
            robotHeadingVariance = robotHeadingVariance + mecanumLocalizer.getLastHeadingVariance();
        }

        if (updateCount % 20 == 0) {
            robotPoseEstimate = new Pose2d(
                    robotPoseEstimate.getTranslation(),
                    new Rotation2d(LocalizationMath.getNormalPDFMean(robotHeadingVariance, robotPoseEstimate.getHeading(), imu.getHeadingVariance(), imu.getUpAxis())));
            robotHeadingVariance = LocalizationMath.getNormalVariance(robotHeadingVariance, imu.getHeadingVariance());
        }

        if (updateCount % 20 != 0) {
            distanceSensorLocalizer.update();
        }


    }

    public CovarianceMatrix getXYCovariance() {
        return robotXYCovariance;
    }

    public double getHeadingVariance() {
        return robotHeadingVariance;
    }

    public double getErrorVelocity() {
        double translationChange = Math.sqrt(Math.pow(robotPoseEstimate.getX() - priorRobotPoseEstimate.getX(),2) + Math.pow(robotPoseEstimate.getY() - priorRobotPoseEstimate.getY(),2));
        double headingChange = robotPoseEstimate.getHeading() - priorRobotPoseEstimate.getHeading();
        double errorVelocity = (translationChange * headingChange) / (updateTime - priorUpdateTime);
        return Math.pow(errorVelocity, integrationErrorConstant);
    }

    @Override
    @NonNull
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return LocalizationMath.toRR(LocalizationMath.mmToIn(robotPoseEstimate));
    }

    @Override
    public void setPoseEstimate(@NonNull com.acmerobotics.roadrunner.geometry.Pose2d newPose) {
        robotPoseEstimate = LocalizationMath.inToMM(LocalizationMath.toFtcLib(newPose));
        priorRobotPoseEstimate = robotPoseEstimate;
        imu.setCurrentAngle(robotPoseEstimate.getHeading());
    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return LocalizationMath.toRR(LocalizationMath.mmToIn( new Pose2d(
                (robotPoseEstimate.getX() - priorRobotPoseEstimate.getX()) / (updateTime - priorUpdateTime),
                (robotPoseEstimate.getY() - priorRobotPoseEstimate.getY()) / (updateTime - priorUpdateTime),
                new Rotation2d((robotPoseEstimate.getHeading() - priorRobotPoseEstimate.getHeading()) / (updateTime - priorUpdateTime)))));
    }
}
