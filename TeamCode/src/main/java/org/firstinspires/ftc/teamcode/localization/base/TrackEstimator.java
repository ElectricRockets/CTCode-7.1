package org.firstinspires.ftc.teamcode.localization.base;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;

public interface TrackEstimator {

    default CovarianceMatrix getTranslationAccuracyEstimate() {
        return new CovarianceMatrix( Integer.MAX_VALUE, Integer.MAX_VALUE, 0);
    }

    default CovarianceMatrix getTranslationAccuracy() {
        return new CovarianceMatrix( Integer.MAX_VALUE, Integer.MAX_VALUE, 0);
    }

    default double getRotationAccuracyEstimate() {
        return Integer.MAX_VALUE;
    }

    default double getRotationAccuracy() {
        return Integer.MAX_VALUE;
    }

    void update();

    Pose2d getPoseEstimate();

    default boolean getValidityTranslation() {
        return false;
    }
    default boolean getValidityRotation() {
        return false;
    }

    default double getReadTime() {return 8.0;}
}
