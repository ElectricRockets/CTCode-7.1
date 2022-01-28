package org.firstinspires.ftc.teamcode.SMARTLocalization.PDFLocalization;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class PDFImu {
    private final BNO055IMU imu;
    private final double staticVariance;
    private final double dynamicVariance;
    private double totalAngularMovement;
    public enum axis {X,Y,Z}
    private final axis upAxis;
    private double imuOffset;

    public PDFImu(BNO055IMU imu, double startAngle, double staticVariance, double dynamicVariance, axis upAxis) {
        this.imu = imu;
        this.staticVariance = staticVariance;
        this.dynamicVariance = dynamicVariance;
        this.upAxis = upAxis;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void addAngularMovement(double angularMovement) {
        totalAngularMovement += Math.abs(angularMovement);
    }

    public double getUpAxis() {
        switch (upAxis) {
            case X: return imu.getAngularOrientation().firstAngle + imuOffset;
            case Y: return imu.getAngularOrientation().secondAngle + imuOffset;
            default: return imu.getAngularOrientation().thirdAngle + imuOffset;
        }
    }

    public double getHeadingVariance() {
        return dynamicVariance * totalAngularMovement + staticVariance;
    }

    public void setCurrentAngle(double newAngle) {
        double currentAngle = getUpAxis();
        imuOffset = newAngle - currentAngle;
    }
}
