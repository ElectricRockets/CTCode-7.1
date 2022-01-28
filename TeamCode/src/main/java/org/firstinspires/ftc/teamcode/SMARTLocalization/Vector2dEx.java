package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class Vector2dEx {

    public static Vector2d getMultipliedVector(Vector2d inputVector, CovarianceMatrix covarianceMatrix) {
        return new Vector2d(
                covarianceMatrix.a * inputVector.getX() + covarianceMatrix.b + inputVector.getY(),
                covarianceMatrix.c * inputVector.getX() + covarianceMatrix.d + inputVector.getY()
        );
    }
}
