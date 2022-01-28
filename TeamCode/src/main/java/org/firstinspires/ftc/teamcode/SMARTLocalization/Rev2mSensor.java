package org.firstinspires.ftc.teamcode.SMARTLocalization;

public class Rev2mSensor {

    public static double getVarianceEstimate(double distanceEstimate, double angleEstimate) {
        return 0;
    }

    public static double readTime(boolean onChub) {
        return onChub ? 6.0 : 8.0;
    }
}
