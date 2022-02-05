package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import java.util.ArrayList;

public class LocalizationMath {

    public static Pose2d localToGlobalPoseChange (Pose2d localChange, double initialAngle) {

        double straitMovement = localChange.getX();
        double lateralMovement = localChange.getY();
        double angleChange = localChange.getHeading();


        double changeXInertial = straitMovement;
        double changeYInertial = lateralMovement;

        if (angleChange != 0) {
            //K is a constant which defines how far the robot would be displaced based off a circular track, and some angular change on that track
            double k = (java.lang.Math.sqrt(2 - 2* java.lang.Math.cos(angleChange)) / angleChange) * java.lang.Math.signum(angleChange);

            //first component is the x component of the strait direction, second is of perpendicular direction
            changeXInertial = (k * straitMovement * java.lang.Math.sin(angleChange)) + (k * lateralMovement * java.lang.Math.cos(angleChange));
            //first component is the y component of the strait direction, second is of perpendicular direction
            changeYInertial = (k * straitMovement * java.lang.Math.cos(angleChange)) + (-k * lateralMovement * java.lang.Math.sin(angleChange));
        }

        //Converts the position change to a vector
        double positionChangeMagnitude = java.lang.Math.hypot(changeXInertial,changeYInertial);
        double positionChangeAngle = java.lang.Math.atan2(changeXInertial,changeYInertial);

        //Rotates the position change vector and converts back to x and y changes
        double positionChangeX = java.lang.Math.cos(positionChangeAngle + initialAngle) * positionChangeMagnitude;
        double positionChangeY = java.lang.Math.sin(positionChangeAngle + initialAngle) * positionChangeMagnitude;

        //outputs Pose2d that can be added to the current position pose
        return new Pose2d(positionChangeX, positionChangeY, new Rotation2d(angleChange));
    }

    public static double getNormalPDFMean(double var1, double avg1, double var2, double avg2) {
        return (avg1/var1 + avg2/var2) / (1/var1 + 1/var2);
    }

    public static double getNormalVariance(double var1, double var2) {
        return var1 * var2/(var1+var2);
    }



    public static Pose2d toFtcLib(com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading()));
    }

    public static com.acmerobotics.roadrunner.geometry.Pose2d toRR(Pose2d pose2d) {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
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

    public static Pose2d getIntersection(Pose2d globalSensorPose, double[] wallOffsets) {

        //creates the perimeter
        addPointToPerimeter(new Vector2d(-1828,-1828));
        addPointToPerimeter(new Vector2d(-1828 - wallOffsets[0], -609));
        addPointToPerimeter(new Vector2d(-1828 - wallOffsets[0], 609));

        addPointToPerimeter(new Vector2d(-1828,1828));
        addPointToPerimeter(new Vector2d(-609, 1828 + wallOffsets[1]));
        addPointToPerimeter(new Vector2d(609, 1828 + wallOffsets[1]));

        addPointToPerimeter(new Vector2d(1828,1828));
        addPointToPerimeter(new Vector2d(1828 + wallOffsets[2], 609));
        addPointToPerimeter(new Vector2d(1828 + wallOffsets[2], -609));

        addPointToPerimeter(new Vector2d(1828,-1828));
        addPointToPerimeter(new Vector2d(609, -1828 - wallOffsets[3]));
        addPointToPerimeter(new Vector2d(-609, -1828 - wallOffsets[3]));
        finalizePerimeter();

        ArrayList<Pose2d> interceptPositions = new ArrayList<>();

        //calculates the closest point of intersection, and adds it to a list if it does have an intersection within the line segment.
        for (int i = 0; i < perimeter.size(); i++) {

            //adds a catch for divide by 0 errors
            double perimeterLineSlope = 1000000000000.0;
            if ((perimeter.get(i).get(0).getY() - perimeter.get(i).get(1).getY()) != 0) {
                perimeterLineSlope = (perimeter.get(i).get(0).getY() - perimeter.get(i).get(1).getY()) / (perimeter.get(i).get(0).getX() - perimeter.get(i).get(1).getX());
            }

            //adds a catch for divide by 0 errors
            double sensorLineSlope = 1000000000000.0;
            if (globalSensorPose.getHeading() % Math.PI > 0.0001) {
                sensorLineSlope = Math.tan(globalSensorPose.getHeading());
            }

            //adds catch for parallel lines
            if (sensorLineSlope != perimeterLineSlope) {

                //calculates the intersection
                double line1Intercept = perimeter.get(i).get(0).getX() * perimeterLineSlope - perimeter.get(i).get(0).getY();
                double line2Intercept = globalSensorPose.getX() * sensorLineSlope - perimeter.get(i).get(0).getY();
                double interceptX = (line2Intercept - line1Intercept)/(perimeterLineSlope - sensorLineSlope);

                boolean withinBoundingBox = (interceptX >= perimeter.get(i).get(0).getX() && interceptX <= perimeter.get(i).get(1).getX()) || (interceptX <= perimeter.get(i).get(0).getX() && interceptX >= perimeter.get(i).get(1).getX());
                boolean withinSensorFaceDirection = (globalSensorPose.getHeading() >= Math.toRadians(0) && interceptX > globalSensorPose.getX()) || (globalSensorPose.getHeading() < Math.toRadians(0) && interceptX < globalSensorPose.getX());

                if (withinBoundingBox && withinSensorFaceDirection) {
                    interceptPositions.add(new Pose2d(interceptX, perimeterLineSlope * interceptX + line1Intercept, new Rotation2d(Math.atan(perimeterLineSlope)) ));
                }
            }
        }

        //iterates through all intersections to find the one closest to the sensor
        Pose2d closestIntersect = new Pose2d();
        for (int i = 0; i < interceptPositions.size(); i++) {
            if (Math.abs(interceptPositions.get(i).getX() - globalSensorPose.getX()) < Math.abs(closestIntersect.getX() - globalSensorPose.getX())) {
                closestIntersect = interceptPositions.get(i);
            }
        }

        return closestIntersect;
    }

    public static Pose2d mmToIn(Pose2d mmPose) {
        return new Pose2d(mmPose.getX() / 25.4, mmPose.getY() / 25.4, mmPose.getRotation());
    }

    public static Pose2d inToMM(Pose2d inPose) {
        return new Pose2d(inPose.getX() * 25.4, inPose.getY() * 25.4, inPose.getRotation());
    }

    public static Pose2d addValues(Pose2d pose1, Pose2d pose2) {
        return new Pose2d(
                pose1.getX() + pose2.getX(),
                pose1.getY() + pose2.getY(),
                new Rotation2d(pose1.getHeading() + pose2.getHeading())
        );
    }
}
