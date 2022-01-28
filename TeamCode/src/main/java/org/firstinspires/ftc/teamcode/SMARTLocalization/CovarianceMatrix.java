package org.firstinspires.ftc.teamcode.SMARTLocalization;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class CovarianceMatrix {

    /*matrix looks like this | a  b |
                             | c  d |
    */
    double a, b, c, d;

    public CovarianceMatrix(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public CovarianceMatrix(double xScale, double yScale, double rotationRad) {
        CovarianceMatrix newMatrix = new CovarianceMatrix(1,0,0,1).scaleBy(xScale, yScale).rotateBy(rotationRad);
        this.a = newMatrix.a;
        this.b = newMatrix.b;
        this.c = newMatrix.c;
        this.d = newMatrix.d;
    }

    public CovarianceMatrix multiplyBy(CovarianceMatrix factor) {
        double a = this.a * factor.a + this.b * factor.c;
        double b = this.a * factor.b + this.b * factor.d;
        double c = this.c * factor.a + this.d * factor.c;
        double d = this.c * factor.b + this.d * factor.d;
        return new CovarianceMatrix(a,b,c,d);
    }

    public CovarianceMatrix add(CovarianceMatrix addend) {
        double a = this.a + addend.a;
        double b = this.b + addend.b;
        double c = this.c + addend.c;
        double d = this.d + addend.d;
        return new CovarianceMatrix(a,b,c,d);
    }

    public CovarianceMatrix subract(CovarianceMatrix subtrahend) {
        double a = this.a - subtrahend.a;
        double b = this.b - subtrahend.b;
        double c = this.c - subtrahend.c;
        double d = this.d - subtrahend.d;
        return new CovarianceMatrix(a,b,c,d);
    }

    public CovarianceMatrix invert() {
        double det = determinant();
        double a = this.d / det;
        double b = -this.b /det;
        double c = -this.c / det;
        double d = this.a / det;
        return new CovarianceMatrix(a,b,c,d);
    }

    public CovarianceMatrix divideBy(CovarianceMatrix divisor) {
        return multiplyBy(divisor.invert());
    }

    public CovarianceMatrix rotateBy(double angRad) {
        return multiplyBy(new CovarianceMatrix(Math.cos(angRad), -Math.sin(angRad), Math.sin(angRad), Math.cos(angRad)));
    }

    public CovarianceMatrix scaleBy(double xScale, double yScale) {
        return multiplyBy(new CovarianceMatrix(xScale, 0, 0, yScale));
    }

    public CovarianceMatrix square() {
        return multiplyBy(new CovarianceMatrix(this.a, this.b, this.c, this.d));
    }

    public CovarianceMatrix combineWith(CovarianceMatrix cM2) {
        //uses the equations here to get the new covariance matrix using matrix notation https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ (15)
        CovarianceMatrix cM1 = new CovarianceMatrix(this.a, this.b, this.c, this.d);
        CovarianceMatrix k = cM1.divideBy(cM1.add(cM2));
        return cM1.subract(cM1.multiplyBy(k));
    }

    public Vector2d getNewMean(Vector2d vec1, CovarianceMatrix cM1, Vector2d vec2, CovarianceMatrix cM2) {
        //uses the equations here to get the new mean using matrix notation https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ (15)
        CovarianceMatrix k = cM1.divideBy(cM1.add(cM2));
        Vector2d subtractedVector = new Vector2d(vec2.getX() - vec1.getX(), vec2.getY() - vec1.getY());
        Vector2d multipliedVector = Vector2dEx.getMultipliedVector(subtractedVector, k);
        return new Vector2d(vec1.getX() + multipliedVector.getX(), vec1.getY() + multipliedVector.getY());
    }

    public double determinant() {
        return this.a * this.d - this.b * this.c;
    }
}
