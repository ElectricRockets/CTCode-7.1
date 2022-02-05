package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;
import org.firstinspires.ftc.teamcode.SMARTLocalization.Vector2dEx;
import org.firstinspires.ftc.teamcode.localization.base.TrackEstimator;
import org.firstinspires.ftc.teamcode.localization.base.TrackUpdater;

public class TrackFuser implements Localizer {

    private Pose2d robotPoseEstimate;
    private CovarianceMatrix robotXYCovariance; //reasonable XY Variance of robot placement
    private double robotHeadingVariance; //reasonable heading variance of robot placement

    private Pose2d priorRobotPoseEstimate;
    private int updateCount; //is -1 that way when the first update happens, all functions that execute on different intervals execute
    private double updateTime;
    private double priorUpdateTime;
    private double extraEstimatorTimeLastLoop;

    private final double INTEGRATION_ERROR = 1.1;

    private TrackUpdater[] trackUpdaters;
    private TrackEstimator[] trackEstimators;

    private Telemetry telemetry;

    public TrackFuser(Telemetry telemetry) {
        this.telemetry = telemetry;
        updateTime = System.nanoTime() * Math.pow(10,-9);
        priorUpdateTime = updateTime - 0.001;
        extraEstimatorTimeLastLoop = 0;
        robotPoseEstimate = new Pose2d(0,0, new Rotation2d(0));
        priorRobotPoseEstimate = robotPoseEstimate;
        robotXYCovariance = new CovarianceMatrix(1,1,0);
        robotHeadingVariance = 0.035;
        updateCount = -1;
    }

    public void setTrackEstimators(TrackEstimator... trackEstimators) {
        this.trackEstimators = trackEstimators;
    }

    public void setTrackUpdaters(TrackUpdater... trackUpdaters) {
        this.trackUpdaters = trackUpdaters;
    }

    public double getRobotHeadingVariance() {
        return robotHeadingVariance;
    }

    @NonNull
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(robotPoseEstimate.getX(), robotPoseEstimate.getY(), robotPoseEstimate.getHeading());
    }

    public Pose2d getFTCLibPoseEstimate() {
        return robotPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        robotPoseEstimate = new Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading()));
    }

    public void setFTCLibPoseEstimate() {

    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return null;
    }

    public Pose2d getFTCLibPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        updateCount++;
        //Pose2d currentVelocity = LocalizationMath.inToMM(LocalizationMath.toFtcLib(getPoseVelocity()));
        priorRobotPoseEstimate = robotPoseEstimate;
        priorUpdateTime = updateTime;
        updateTime = System.nanoTime() * Math.pow(10,-9);
        double rawLoopTime = updateTime - priorUpdateTime - extraEstimatorTimeLastLoop;

        boolean completedUpdate = false;
        CovarianceMatrix trackUpdaterCovariance = new CovarianceMatrix(Integer.MAX_VALUE, Integer.MAX_VALUE, 0);
        double trackUpdaterHeadingVariance = Integer.MAX_VALUE;

        for (int i = 0; i < trackUpdaters.length && !completedUpdate; i++) {
            if (trackUpdaters[i].getValidity()) {
                trackUpdaters[i].update();

                robotPoseEstimate = LocalizationMath.addValues(
                        robotPoseEstimate,
                        LocalizationMath.localToGlobalPoseChange(trackUpdaters[i].getLocalPoseUpdate(), robotPoseEstimate.getHeading()));
                robotXYCovariance = robotXYCovariance.add(trackUpdaters[i].getCovarianceUpdate());
                trackUpdaterCovariance = trackUpdaters[i].getCovarianceUpdate();
                robotHeadingVariance = robotHeadingVariance + trackUpdaters[i].getHeadingVarianceUpdate();
                trackUpdaterHeadingVariance = trackUpdaters[i].getHeadingVarianceUpdate();
                completedUpdate = true;
            }
        }

        //this code determines the best sensor to use, based off of the resulting improvement in accuracy relative to reading the updater more rapidly
        int optimalEstimator = -1;
        double optimalScore = 0;

        if (trackEstimators != null) {
            for (int i = 0; i < trackEstimators.length; i++) {
                double withDelayScale = Math.pow((rawLoopTime + trackEstimators[i].getReadTime()) / rawLoopTime, INTEGRATION_ERROR);
                CovarianceMatrix withDelayCovariance = trackUpdaterCovariance.multiplyBy(new CovarianceMatrix(withDelayScale, withDelayScale, 0));
                double withoutDelayScale = (rawLoopTime + trackEstimators[i].getReadTime()) / rawLoopTime;
                CovarianceMatrix withoutDelayCovariance = trackUpdaterCovariance.multiplyBy(new CovarianceMatrix(withoutDelayScale, withoutDelayScale, 0));

                CovarianceMatrix robotWithEstimatorCovariance = robotXYCovariance.add(withDelayCovariance);
                CovarianceMatrix robotWithoutEstimatorCovariance = robotXYCovariance.add(withoutDelayCovariance);

                robotWithEstimatorCovariance = robotWithEstimatorCovariance.combineWith(trackEstimators[i].getTranslationAccuracyEstimate());

                double withEstimatorHeadingVariance = LocalizationMath.getNormalVariance(robotHeadingVariance, trackEstimators[i].getRotationAccuracyEstimate());

                double scoreWithEstimator = robotWithEstimatorCovariance.determinant() * Math.pow(withEstimatorHeadingVariance, 2);
                double scoreWithoutEstimator = robotWithoutEstimatorCovariance.determinant() * Math.pow(robotHeadingVariance, 2);

                double EstimatorScore = scoreWithEstimator - scoreWithoutEstimator;

                if (!trackEstimators[i].getValidity()) {
                    EstimatorScore = 0;
                }

                if (EstimatorScore < optimalScore) {
                    optimalEstimator = i;
                    optimalScore = EstimatorScore;
                }

            }
        }
        //utilizes the optimal sensor to optimize the position estimate
        if (optimalEstimator != -1) {
            trackEstimators[optimalEstimator].update();
            CovarianceMatrix trackEstimatorCovariance = trackEstimators[optimalEstimator].getTranslationAccuracy();
            double trackEstimatorHeadingVariance = trackEstimators[optimalEstimator].getRotationAccuracy();
            Pose2d trackEstimatorPose = trackEstimators[optimalEstimator].getPoseEstimate();

            CovarianceMatrix K = robotXYCovariance.divideBy(robotXYCovariance.add(trackEstimatorCovariance));
            Vector2d robotPoseEstimateVector = new Vector2d(robotPoseEstimate.getX(), robotPoseEstimate.getY());
            Vector2d trackEstimatorVector = new Vector2d(trackEstimatorPose.getX(), trackEstimatorPose.getY());

            Vector2d newPoseVector = robotPoseEstimateVector.plus(Vector2dEx.getMultipliedVector(trackEstimatorVector.minus(robotPoseEstimateVector), K ));

            robotXYCovariance = robotXYCovariance.combineWith(trackEstimatorCovariance);
            double trackEstimatorHeading = trackEstimatorPose.getHeading();

            double robotHeadingEstimate = LocalizationMath.getNormalPDFMean(robotHeadingVariance, robotPoseEstimate.getHeading(), trackEstimatorHeadingVariance, trackEstimatorHeading);

            robotHeadingVariance = LocalizationMath.getNormalVariance(robotHeadingVariance, trackEstimatorHeadingVariance);

            robotPoseEstimate = new Pose2d(newPoseVector.getX(), newPoseVector.getY(), new Rotation2d(robotHeadingEstimate));
        } else {
            extraEstimatorTimeLastLoop = 0;
        }


    }
}
