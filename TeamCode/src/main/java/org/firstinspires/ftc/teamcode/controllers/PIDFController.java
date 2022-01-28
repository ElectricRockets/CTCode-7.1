package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

public class PIDFController {

    DoubleSupplier positionSupplier, setPointSupplier, kF, kS;
    double kP, kI, kD, dSmoothing, kA, min, max, pastSetPoint, pastPosition, pastIntegral, pastVelocity, pastExecutionTime;

    public PIDFController(DoubleSupplier positionSupplier, DoubleSupplier setPointSupplier, double kP, double kI, double kD, double dSmoothing, DoubleSupplier kF, DoubleSupplier kS, double kA, double min, double max) {
        this.positionSupplier = positionSupplier;
        this.setPointSupplier = setPointSupplier;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.dSmoothing = dSmoothing;
        this.kF = kF;
        this.kS = kS;
        this.kA = kA;
        this.min = min;
        this.max = max;
        pastVelocity = 0;
        pastIntegral = 0;
    }

    public double getUpdate() {
        double position = positionSupplier.getAsDouble();
        double setPoint = setPointSupplier.getAsDouble();
        double error = setPoint - position;
        double executionTime = System.nanoTime() / Math.pow(10,9);

        double loopTime = executionTime - pastExecutionTime;
        double velocity = ((position - pastPosition) / loopTime);

        double oP = kP * error;
        double oI = kI * error + pastIntegral;
        //double oD = kD * (position - pastPosition) / loopTime;
        double oD = kD * (velocity * (1 / dSmoothing) + pastVelocity * (1 - (1 / dSmoothing)));
        double oF = kF.getAsDouble() * Math.signum(error);
        double oS = kS.getAsDouble();
        double oA = kA * (setPoint - pastSetPoint) * Math.signum(error) * loopTime;

        double output = oP + oI + oD + oF + oS + oA;
        boolean outOfRange = output > max || output < min;
        if (outOfRange) {
            output = Range.clip(output, min, max);
        } else {
            pastIntegral = oI;
        }

        pastSetPoint = setPoint;
        pastPosition = position;
        pastExecutionTime = executionTime;
        pastVelocity = ((position - pastPosition) / loopTime);

        return output;
    }
}