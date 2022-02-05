package org.firstinspires.ftc.teamcode.SMARTLocalization;

public class Rev2mSensor {

    public static double getVarianceEstimate(double distanceEstimate, double angleEstimate) {
        if (Math.abs(angleEstimate - Math.toRadians(90)) > 0.26) {
            return Integer.MAX_VALUE;
        }
        return 0.0115 * Math.pow(Math.E, 0.0992 * distanceEstimate);
    }

    public static double readTime(boolean onChub) {
        return onChub ? 6.0 : 8.0;
    }
}
