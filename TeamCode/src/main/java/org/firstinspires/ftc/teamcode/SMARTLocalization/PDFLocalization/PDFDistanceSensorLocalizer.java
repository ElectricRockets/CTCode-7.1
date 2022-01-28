package org.firstinspires.ftc.teamcode.SMARTLocalization.PDFLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.DistanceSensorEx;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PDFDistanceSensorLocalizer {

    private final ArrayList<DistanceSensorEx> distanceSensors = new ArrayList<>();
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<CovarianceMatrix> robotCovarianceSupplier;
    private final DoubleSupplier robotErrorVelocitySupplier;
    private Pose2d newPoseEstimate;
    private double[] wallOffsets;

    public PDFDistanceSensorLocalizer(Supplier<Pose2d> robotPoseSupplier, Supplier<CovarianceMatrix> robotCovarianceSupplier, DoubleSupplier robotErrorVelocitySupplier) {
        this.robotCovarianceSupplier = robotCovarianceSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotErrorVelocitySupplier = robotErrorVelocitySupplier;
        newPoseEstimate = new Pose2d(0,0,new Rotation2d(0));
        wallOffsets = new double[] {0.0,0.0,0.0,0.0};
    }

    public void setWallOffsets(double[] newWallOffsets) {
        wallOffsets = newWallOffsets;
    }

    public void add(DistanceSensorEx newDistanceSensor) {
        distanceSensors.add(newDistanceSensor);
    }

    public void update() {
        Pose2d robotPose = robotPoseSupplier.get();
        CovarianceMatrix robotCovariance = robotCovarianceSupplier.get();
        double robotErrorVelocity = robotErrorVelocitySupplier.getAsDouble();

        int optimalSensor = -1;
        double optimalScore = 0;
        Pose2d optimalIntersection = new Pose2d();

        //calculates the score for each sensor based off of approximated information
        for(int i = 0 ; i < distanceSensors.size(); i++) {
            Pose2d globalSensorPose = distanceSensors.get(i).getSensorPose().relativeTo(robotPose);
            Pose2d intersectionPose = LocalizationMath.getIntersection(globalSensorPose, wallOffsets); //gets xy of intersection along with the heading of the perimeter line the intersection is on

            //calculates an estimate for the sensor variance in detecting the distance to the wall
            double distanceEstimate = Math.hypot(globalSensorPose.getX() - intersectionPose.getX(), globalSensorPose.getY() - intersectionPose.getY());
            double angleEstimate = (intersectionPose.getHeading() - globalSensorPose.getHeading()) % (Math.PI/2);
            double varianceMultiplier = Math.sin(intersectionPose.getHeading());
            double correctionVariance = distanceSensors.get(i).getVarianceEstimate(distanceEstimate, angleEstimate) * varianceMultiplier;

            //gets a covariance matrix for the sensors robot position estimate accuracy, then combines with robot covariance to get a number for the total variance of robot position in the end.
            CovarianceMatrix sensorCovariance = new CovarianceMatrix(correctionVariance, 1000000.0, intersectionPose.getHeading());
            double totalCovarianceDeterminant = robotCovariance.combineWith(sensorCovariance).determinant();


            double score = 1.0;
            if (score > optimalScore) {
                optimalSensor = i;
                optimalScore = score;
                optimalIntersection = intersectionPose;
            }
        }

        double correctionFactor = 1;
        double estimatedDist = Math.hypot(distanceSensors.get(optimalSensor).getSensorPose().getX() - optimalIntersection.getX(), distanceSensors.get(optimalSensor).getSensorPose().getY() - optimalIntersection.getY());
        double error = (distanceSensors.get(optimalSensor).getDistance() - estimatedDist)
                * Math.sin(optimalIntersection.getHeading() - distanceSensors.get(optimalSensor).getSensorPose().getHeading());

        newPoseEstimate = new Pose2d(
                error * correctionFactor * Math.cos(optimalIntersection.getHeading()),
                error * correctionFactor * Math.sin(optimalIntersection.getHeading()),
                new Rotation2d(0)
        );
    }

    public Pose2d getNewPose() {
        return newPoseEstimate;
    }
}
