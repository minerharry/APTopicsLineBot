package org.firstinspires.ftc.teamcode.Pipelines;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.ORB;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * GreenNodeFinder class.
 *
 * <p>An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class GreenNodeFinder extends BetterOpenCVPipeline {

    //Outputs
    private Mat hsvThresholdOutput = new Mat();
    private Mat maskOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();
    private Point findNodeCenter = new Point();
    private double findNodeArea = 0;
    private double findNodeRadius = 0;
    private boolean usingTelemetry = false;
    private Telemetry telemetry;
    private Mat findNodeImageOutput = new Mat();
    private Mat allNodeImageOutput = new Mat();
    private Mat outputMat = new Mat();


    private enum Stage {
        HSV_THRESHOLD,
        CONTOURS,
        CONVEX_HULLS,
        NODE_DRAWN,
        ALL_NODES, //above a certain area
        RAW_IMAGE
    }

    protected Stage stageToRenderToViewport = Stage.ALL_NODES;
    private Stage[] stages = Stage.values();

    public  GreenNodeFinder(){}

    @Override
    public Mat processImage(Mat input) {
        outputTelemetry("Stage: ", stageToRenderToViewport.name());
        return process(input);
    }

    public GreenNodeFinder(Telemetry externalTelemetry) {
        usingTelemetry = true;
        telemetry = externalTelemetry;
    }

    private void outputTelemetry(String caption, String msg){
        if (usingTelemetry){
            telemetry.addData(caption,msg);
        }
    }




    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat process(Mat source0) {



        // Step HSV_Threshold0:
        Mat hsvThresholdInput = source0;
        double[] hsvThresholdHue = {39.74819912327278, 97.06484641638228};
        double[] hsvThresholdSaturation = {47.39208323278015, 255.0};
        double[] hsvThresholdValue = {35.51798451804429, 240};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Mask0:
        Mat maskInput = source0;
        Mat maskMask = hsvThresholdOutput;
        mask(maskInput, maskMask, maskOutput);

        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = true;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        // Step Convex_Hulls0:
        ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
        convexHulls(convexHullsContours, convexHullsOutput);

        // Find most circular contour by comparing minEnclosingCircle area to contour area
        double areaLimit = 1500;
        double minCircularity = 0.15;
        double radius = 0;
        Point center = null;
        double maxArea = 0;
        double maxCircularity = 0 ;

        Mat compare_image1 = new Mat();
        Mat compare_image2 = new Mat();
        Mat compare_out = new Mat();
        Mat compare_out2 = new Mat();
        MatOfPoint c = new MatOfPoint();
        MatOfPoint2f c2f= new MatOfPoint2f();
        ArrayList<double[][]> validNodes = new ArrayList<>();
        Mat bitwise_image = new Mat(maskInput.rows(),maskInput.cols(),maskInput.type());
        for (int i = 0; i < convexHullsOutput.size(); i++) {
            c = convexHullsOutput.get(i);
            double contour_area = Imgproc.contourArea(c);
            if (contour_area > areaLimit) {
                compare_image1 = new Mat();
                compare_image2 = new Mat();
                bitwise_image.copyTo(compare_image1);
                bitwise_image.copyTo(compare_image2);

                c2f = new MatOfPoint2f(c.toArray());
                float[] circleRadiusArray = new float[1];
                Point circleCenter = new Point();
                Imgproc.minEnclosingCircle(c2f, circleCenter, circleRadiusArray);
                double circleRadius = circleRadiusArray[0];


                Imgproc.drawContours(compare_image1, convexHullsOutput, i, new Scalar(255, 255, 255), -1);
                Imgproc.circle(compare_image2, circleCenter, (int) (circleRadius), new Scalar(255, 255, 255), -1);
                compare_out = new Mat();
                Core.bitwise_and(compare_image1, compare_image2, compare_out);
                compare_out2 = new Mat();
                Imgproc.cvtColor(compare_out,compare_out2,Imgproc.COLOR_RGB2GRAY);



                double circularity = Core.countNonZero(compare_out2) / (Math.PI * 2 * circleRadius * circleRadius);

                if (circularity > minCircularity) {
                    double[][] tempData = {{circleRadius},{circleCenter.x,circleCenter.y}};
                    validNodes.add(tempData);
                }
                if (circularity > maxCircularity) {
                    maxCircularity = circularity;
                    maxArea = contour_area;
                    center = circleCenter;
                    radius = circleRadius;
                }

            }
            compare_image1.release();
            compare_image2.release();
            compare_out.release();
            compare_out2.release();
        }


        compare_image1.release();
        compare_image2.release();
        compare_out.release();
        compare_out2.release();

        findNodeArea = maxArea;

        findNodeCenter = center;
        findNodeRadius = radius;

        //Draw node circle on output image
        findNodeImageOutput = new Mat();
        source0.copyTo(findNodeImageOutput);
        if (findNodeCenter != null)
            Imgproc.circle(findNodeImageOutput,findNodeCenter,(int)findNodeRadius,new Scalar(0,0,255));


        allNodeImageOutput = new Mat();
        source0.copyTo(allNodeImageOutput);
        for (double[][] circle : validNodes) {
            Imgproc.circle(allNodeImageOutput,new Point(circle[1][0],circle[1][1]) ,(int)circle[0][0],new Scalar(0,0,255));
        }

        outputTelemetry("Convex hulls: ", ""+convexHullsOutput);
        for (MatOfPoint contour : convexHullsOutput) {
            if (contour.empty()) {
                System.out.println("Error: Empty convex hull vector");
            }
        }

        outputTelemetry("Contours: ", ""+findContoursOutput);
        for (MatOfPoint contour : findContoursOutput) {
            if (contour.empty()) {
                System.out.println("Error: Empty Contour vector");
            }
        }

        //System.out.println("Convex hulls: " + convexHullsOutput);



        outputMat = new Mat();
        source0.copyTo(outputMat);
        switch (stageToRenderToViewport) {
            case ALL_NODES:
                return allNodeImageOutput;
            case RAW_IMAGE:
                return source0;
            case CONTOURS:
                Imgproc.drawContours(outputMat,findContoursOutput,-1,new Scalar(0,0,255));
                return outputMat;
            case NODE_DRAWN:
                outputMat = findNodeImageOutput;
                return outputMat;
            case CONVEX_HULLS:
                outputTelemetry("Convex hulls: ", ""+convexHullsOutput);
                //System.out.println(convexHullsOutput);
                Imgproc.drawContours(outputMat,convexHullsOutput,-1,new Scalar(0,0,255));
                return outputMat;
            case HSV_THRESHOLD:
                return maskOutput;
        }
        return source0;
    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a Mask.
     * @return Mat output from Mask.
     */
    public Mat maskOutput() {
        return maskOutput;
    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }

    /**
     * This method is a generated getter for the output of a Convex_Hulls.
     * @return ArrayList<MatOfPoint> output from Convex_Hulls.
     */
    public ArrayList<MatOfPoint> convexHullsOutput() {
        return convexHullsOutput;
    }

    public Point getFindNodeCenter() {
        return findNodeCenter;
    }

    public double getFindNodeRadius() {
        return findNodeRadius;
    }

    public Mat getFindNodeImageOutput() {
        return findNodeImageOutput;
    }

    public double getFindNodeArea() {
        return findNodeArea;
    }






    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input The image on which to perform the Distance Transform.

     */
    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
     * Compute the convex hulls of contours.
     * @param inputContours The contours on which to perform the operation.
     * @param outputContours The contours where the output will be stored.
     */
    private void convexHulls(List<MatOfPoint> inputContours,
                             ArrayList<MatOfPoint> outputContours) {
        final MatOfInt hull = new MatOfInt();
        outputContours.clear();
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            outputContours.add(mopHull);
        }
    }


    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public String getDisplayedStageName() {
        return "Green Node finder, displayed stage: " + stageToRenderToViewport.name();
    }

}

