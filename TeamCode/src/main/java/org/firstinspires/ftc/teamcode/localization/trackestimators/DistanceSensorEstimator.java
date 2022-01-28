package org.firstinspires.ftc.teamcode.localization.trackestimators;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SMARTLocalization.CovarianceMatrix;
import org.firstinspires.ftc.teamcode.SMARTLocalization.LocalizationMath;
import org.firstinspires.ftc.teamcode.SMARTLocalization.Rev2mSensor;
import org.firstinspires.ftc.teamcode.localization.base.TrackEstimator;

import java.util.ArrayList;
import java.util.function.Supplier;

public class DistanceSensorEstimator implements TrackEstimator {

    private final Pose2d sensorPose;
    private final DistanceSensor sensor;
    public enum DSTypes {REV2M}
    private final DSTypes dSType;
    private final boolean onChub;
    private final Supplier<Pose2d> robotPoseSupplier;
    private static double[] wallOffsets = new double[] {0,0,0,0};
    private CovarianceMatrix translationAccuracy;
    private Pose2d robotPoseEstimate;

    private double estimatedIntersectDistance = -1;
    private double estimatedIntersectAngle = 0;
    private double estimatedCorrectionAngle = 0;

    public DistanceSensorEstimator(Supplier<Pose2d> robotPoseSupplier, Pose2d sensorPose, DSTypes dSType, boolean onChub, DistanceSensor sensor, double[] wallOffsets) {
        this.sensorPose = sensorPose;
        this.sensor = sensor;
        this.dSType = dSType;
        this.onChub = onChub;
        this.robotPoseSupplier = robotPoseSupplier;

        addPointToPerimeter(new Vector2d(-71,-71));
        addPointToPerimeter(new Vector2d(-71 - wallOffsets[1], -23.6));
        addPointToPerimeter(new Vector2d(-71 - wallOffsets[1], 23.6));

        addPointToPerimeter(new Vector2d(-71,71));
        addPointToPerimeter(new Vector2d(-23.6, 71 + wallOffsets[2]));
        addPointToPerimeter(new Vector2d(23.6, 71 + wallOffsets[2]));

        addPointToPerimeter(new Vector2d(71,71));
        addPointToPerimeter(new Vector2d(71 + wallOffsets[3], 23.6));
        addPointToPerimeter(new Vector2d(71 + wallOffsets[3], -23.6));

        addPointToPerimeter(new Vector2d(71,-71));
        addPointToPerimeter(new Vector2d(23.6, -71 - wallOffsets[4]));
        addPointToPerimeter(new Vector2d(-23.6, -71 - wallOffsets[4]));
        finalizePerimeter();
    }


    @Override
    public CovarianceMatrix getTranslationAccuracyEstimate() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d globalSensorPose = sensorPose.relativeTo(robotPose);
        Pose2d intersectionPose = LocalizationMath.getIntersection(globalSensorPose, wallOffsets); //gets xy of intersection along with the heading of the perimeter line the intersection is on

        //calculates an estimate for the sensor variance in detecting the distance to the wall
        double distanceEstimate = Math.hypot(globalSensorPose.getX() - intersectionPose.getX(), globalSensorPose.getY() - intersectionPose.getY());
        double angleEstimate = (intersectionPose.getHeading() - globalSensorPose.getHeading()) % (Math.PI/2);
        double varianceMultiplier = Math.sin(intersectionPose.getHeading());
        double correctionVariance = getVarianceEstimate(distanceEstimate, angleEstimate) * varianceMultiplier;

        //gets a covariance matrix for the sensors robot position estimate accuracy, then combines with robot covariance to get a number for the total variance of robot position in the end.
        return new CovarianceMatrix(correctionVariance, Integer.MAX_VALUE, intersectionPose.getHeading());
    }

    @Override
    public CovarianceMatrix getTranslationAccuracy() {
        return translationAccuracy;
    }

    @Override
    public void update() {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d globalSensorPose = sensorPose.relativeTo(robotPose);
        calculateIntersectEstimate(globalSensorPose);

        //calculates an estimate for the sensor variance in detecting the distance to the wall
        double distanceReading = sensor.getDistance(DistanceUnit.INCH);
        double correctionVariance = getVarianceEstimate(distanceReading, estimatedIntersectAngle) * Math.sin(estimatedIntersectAngle);

        //gets a covariance matrix for the sensors robot position estimate accuracy, then combines with robot covariance to get a number for the total variance of robot position in the end.
        translationAccuracy = new CovarianceMatrix(correctionVariance, Integer.MAX_VALUE, estimatedCorrectionAngle);
        Pose2d robotPoseEstimateError = new Pose2d( (distanceReading - estimatedIntersectDistance) * Math.sin(estimatedIntersectAngle), 0, new Rotation2d(estimatedCorrectionAngle));
        robotPoseEstimate = robotPoseEstimateError.relativeTo(robotPose);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return robotPoseEstimate;
    }

    @Override
    public boolean getValidityTranslation() {
        return true;
    }

    public double getVarianceEstimate(double distanceEstimate, double angleEstimate) {
        if (dSType == DSTypes.REV2M) {
            return Rev2mSensor.getVarianceEstimate(distanceEstimate, angleEstimate);
        }

        return Integer.MAX_VALUE; //returns a nearly infinite value if the sensor type is unknown
    }

    private static ArrayList<ArrayList<Vector2d>> perimeter = new ArrayList<>();

    private static void addPointToPerimeter (Vector2d newPoint) {
        if (perimeter.isEmpty()) {
            ArrayList<Vector2d> seg = new ArrayList<>();
            seg.add(newPoint);
            perimeter.add(seg);
        } else if (perimeter.size() == 1) {
            perimeter.get(0).add(newPoint);
        } else {
            ArrayList<Vector2d> seg = new ArrayList<>();
            seg.add(perimeter.get(perimeter.size() - 1).get(1));
            seg.add(newPoint);
            perimeter.add(seg);
        }
    }

    private static void finalizePerimeter () {
        ArrayList<Vector2d> seg = new ArrayList<>();
        seg.add(perimeter.get(perimeter.size() - 1).get(1));
        seg.add(perimeter.get(0).get(0));
        perimeter.add(seg);
    }

    public void calculateIntersectEstimate(Pose2d globalSensorPose) {

        double intersectDistance = Integer.MAX_VALUE;
        double intersectAngle = 0;
        double correctionAngle = 0;

        //calculates the closest point of intersection, and adds it to a list if it does have an intersection within the line segment.
        for (int i = 0; i < perimeter.size(); i++) {

            double intersectX = Integer.MAX_VALUE;
            double intersectY = Integer.MAX_VALUE;

            if ((perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX()) != 0 || !((globalSensorPose.getHeading() + Math.PI) % (Math.PI * 2) > Float.MIN_VALUE)) {
                if ((perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX()) == 0) {
                    double sensorLineSlope = Math.tan(globalSensorPose.getHeading());
                    intersectX = perimeter.get(i).get(0).getX();
                    intersectY = sensorLineSlope * (intersectX - globalSensorPose.getX()) + globalSensorPose.getY();

                } else if ((globalSensorPose.getHeading() + Math.PI / 2) % (Math.PI) > Float.MIN_VALUE) {
                    double perimeterLineSlope = (perimeter.get(i).get(0).getY() - perimeter.get(i).get(1).getY()) / (perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX());
                    intersectX = globalSensorPose.getX();
                    intersectY = perimeterLineSlope * (intersectX - perimeter.get(i).get(0).getX()) + perimeter.get(i).get(0).getY();

                } else {
                    double perimeterLineSlope = (perimeter.get(i).get(0).getY() - perimeter.get(i).get(1).getY()) / (perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX());
                    double sensorLineSlope = Math.tan(globalSensorPose.getHeading());

                    if (sensorLineSlope != perimeterLineSlope) {
                        intersectX = ( perimeterLineSlope - perimeter.get(i).get(0).getY() - sensorLineSlope + globalSensorPose.getY() ) / (perimeterLineSlope - sensorLineSlope);
                        intersectY = sensorLineSlope * (intersectX - globalSensorPose.getX()) + globalSensorPose.getY();
                    }
                }
            }

            if (intersectX != Integer.MAX_VALUE) {
                boolean withinBoundingX = (intersectX >= perimeter.get(i).get(0).getX() && intersectX <= perimeter.get(i).get(1).getX()) || (intersectX <= perimeter.get(i).get(0).getX() && intersectX >= perimeter.get(i).get(1).getX());
                boolean withinBoundingY = (intersectY >= perimeter.get(i).get(0).getY() && intersectY <= perimeter.get(i).get(1).getY()) || (intersectY <= perimeter.get(i).get(0).getY() && intersectY >= perimeter.get(i).get(1).getY());
                boolean withinSensorFaceDirection =
                        globalSensorPose.getHeading() >= Math.toRadians(-90) && globalSensorPose.getHeading() <= Math.toRadians(90) && intersectX > globalSensorPose.getX() ||
                                !(globalSensorPose.getHeading() >= Math.toRadians(-90) && globalSensorPose.getHeading() <= Math.toRadians(90)) && intersectX < globalSensorPose.getX();
                double distance = Math.hypot(intersectX - globalSensorPose.getX(), intersectY - globalSensorPose.getY());

                if (withinBoundingX && withinBoundingY && withinSensorFaceDirection && distance < intersectDistance) {
                    double wallAngle;
                    if (perimeter.get(i).get(0).getX() == perimeter.get(i).get(1).getX()) {
                        wallAngle = Math.PI / 2;
                    } else {
                        wallAngle = Math.atan((perimeter.get(i).get(0).getY() - perimeter.get(i).get(1).getY()) / (perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX()));
                    }
                    double wallSensorAngle = Math.atan((globalSensorPose.getY() - perimeter.get(i).get(0).getY()) / (globalSensorPose.getX() - perimeter.get(i).get(0).getX()));

                    boolean sensorToWallRight = wallAngle > wallSensorAngle;

                    intersectAngle = globalSensorPose.getHeading() + wallAngle;
                    if (intersectAngle > Math.PI / 2) {
                        intersectAngle = -(intersectAngle - Math.PI / 2);
                    }

                    correctionAngle = wallAngle + (sensorToWallRight ? 90 : -90);

                    intersectDistance = distance;
                }
            }
        }
        estimatedCorrectionAngle = correctionAngle;
        estimatedIntersectDistance = intersectDistance;
        estimatedIntersectAngle = intersectAngle;
    }
}
