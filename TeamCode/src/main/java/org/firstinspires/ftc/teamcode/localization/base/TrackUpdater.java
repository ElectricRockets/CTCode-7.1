package org.firstinspires.ftc.teamcode.localization.base;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;

public interface TrackUpdater {

    void update();

    Pose2d getLocalPoseUpdate();

    CovarianceMatrix getCovarianceUpdate();

    double getHeadingVarianceUpdate();

    boolean getValidity();

    boolean getCalculateIfInvalid();
}
