package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config
public class TSEPipeline extends OpenCvPipeline
{
    public enum tsePositions {LEFT, MID, RIGHT}
    public tsePositions lastResult;
    private static int scale = 2;
    public static int leftMatrixX = 2/scale;
    public static int leftMatrixY = 80/scale;
    public static int leftMatrixW = 200/scale;
    public static int leftMatrixH = 100/scale;
    public static int middleMatrixX = 540/scale;
    public static int middleMatrixY = 80/scale;
    public static int middleMatrixW = 200/scale;
    public static int middleMatrixH = 100/scale;
    public static int rightMatrixX = 1078/scale;
    public static int rightMatrixY = 80/scale;
    public static int rightMatrixW = 200/scale;
    public static int rightMatrixH = 100/scale;

    public Scalar low1 = new Scalar(0,0,0);
    public Scalar low2 = new Scalar(150,0,0);
    public Scalar high1 = new Scalar(10,255,255);
    public Scalar high2 = new Scalar(180,255,255);

    public static double satFactor = 1.5;
    public static double valueFactor = 0.6;

    private Mat workingMatrix = new Mat();
    private Mat lowH = new Mat();
    private Mat highH = new Mat();
    private Mat leftMat = new Mat();
    private Mat midMat = new Mat();
    private Mat rightMat = new Mat();
    private Mat viewMat = new Mat();

    private final Telemetry telemetry;

    public TSEPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) {
            return input;
        }

        //crops and converts the image into HSV
        input = input.submat(230, 360, 0, 640);
        input.copyTo(workingMatrix);
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV);

        //gets averages to use in dynamic thresholding
        double averageSaturation = Core.sumElems(workingMatrix).val[1] / (workingMatrix.rows() * workingMatrix.cols());
        double averageValue = Core.sumElems(workingMatrix).val[2] / (workingMatrix.rows() * workingMatrix.cols());

        //thresholds the workingMatrix, it needs multiple operations because red is either a high or low hue
        Core.inRange(workingMatrix, new Scalar(low1.val[0], satFactor * averageSaturation, valueFactor * averageValue), high1, lowH);
        Core.inRange(workingMatrix, new Scalar(low2.val[0], satFactor * averageSaturation, valueFactor * averageValue), high2, highH);
        Core.bitwise_or(lowH, highH, workingMatrix);

        //creates the 3 detection spaces
        leftMat = workingMatrix.submat( leftMatrixY, leftMatrixY + leftMatrixH, leftMatrixX, leftMatrixX + leftMatrixW);
        midMat = workingMatrix.submat(middleMatrixY, middleMatrixY + middleMatrixH, middleMatrixX, middleMatrixX + middleMatrixW);
        rightMat = workingMatrix.submat(rightMatrixY, rightMatrixY + rightMatrixH, rightMatrixX, rightMatrixX + rightMatrixW);

        //calculates average value in each detection space
        double leftAvg = Core.sumElems(leftMat).val[0] / (leftMat.rows() * leftMat.cols());
        double midAvg = Core.sumElems(midMat).val[0] / (midMat.rows() * midMat.cols());
        double rightAvg = Core.sumElems(rightMat).val[0] / (rightMat.rows() * rightMat.cols());

        //uses the highest value as the best option for detection
        double max = Math.max(leftAvg, Math.max(midAvg, rightAvg));
        if (rightAvg == max) {
            lastResult = tsePositions.RIGHT;
        }
        else if (leftAvg == max) {
            lastResult = tsePositions.LEFT;
        }
        else {
            lastResult = tsePositions.MID;
        }

        //creates a display version of the image that shows what is recognised
        input.copyTo(viewMat);
        Core.bitwise_not(input, viewMat, workingMatrix);

        Imgproc.rectangle(viewMat, new Rect(leftMatrixX,leftMatrixY,leftMatrixW,leftMatrixH), new Scalar(255,0,0));
        Imgproc.rectangle(viewMat, new Rect(middleMatrixX,middleMatrixY,middleMatrixW, middleMatrixH), new Scalar(255,0,0));
        Imgproc.rectangle(viewMat, new Rect(rightMatrixX,rightMatrixY,rightMatrixW, rightMatrixH), new Scalar(255,0,0));

        leftMat.release();
        rightMat.release();
        midMat.release();
        input.release();
        workingMatrix.release();
        lowH.release();
        highH.release();


        return viewMat;
    }

    public tsePositions getLatestResults() {
        return lastResult;
    }

}
