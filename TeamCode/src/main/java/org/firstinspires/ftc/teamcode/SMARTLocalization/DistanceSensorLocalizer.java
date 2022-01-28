package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.function.Supplier;

public class DistanceSensorLocalizer {

    private final ArrayList<DistanceSensorEx> distanceSensors = new ArrayList<>();
    private final Supplier<Pose2d> robotPoseSupplier;
    private Pose2d correctionPose;
    private double[] wallOffsets;

    public DistanceSensorLocalizer(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        correctionPose = new Pose2d(0,0,new Rotation2d(0));
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

        int optimalSensor = -1;
        double optimalScore = 0;
        Pose2d optimalIntersection = new Pose2d();

        //calculates the score for each sensor based off of approximated information
        for(int i = 0 ; i < distanceSensors.size(); i++) {
            Pose2d globalSensorPose = distanceSensors.get(i).getSensorPose().relativeTo(robotPose);
            Pose2d intersectionPose = LocalizationMath.getIntersection(globalSensorPose, wallOffsets); //gets xy of intersection along with the heading of the perimeter line the intersection is on
            double correctionSx = Math.sin(intersectionPose.getHeading()) * distanceSensors.get(i).getVarianceEstimate(
                    Math.hypot(globalSensorPose.getX() - intersectionPose.getX(), globalSensorPose.getY() - intersectionPose.getY()),
                    (intersectionPose.getHeading() - globalSensorPose.getHeading()) % (Math.PI/2));
            double score = 1.0 / correctionSx;
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

        correctionPose = new Pose2d(
                error * correctionFactor * Math.cos(optimalIntersection.getHeading()),
                error * correctionFactor * Math.sin(optimalIntersection.getHeading()),
                new Rotation2d(0)
        );
    }

    public Pose2d getPoseCorrection() {
        return correctionPose;
    }
}
