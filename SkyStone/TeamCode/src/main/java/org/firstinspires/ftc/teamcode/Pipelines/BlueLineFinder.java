package org.firstinspires.ftc.teamcode.Pipelines;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * GripPipeline class.
 *
 * <p>
 * An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class BlueLineFinder extends BetterOpenCVPipeline {


    private double[] hsvThresholdHue = new double[]{98.21167173282997, 139.51597296375203};
    private double[] hsvThresholdSaturation = new double[]{101.83093962566458, 255.0};
    private double[] hsvThresholdValue = new double[]{110, 255.0};


    private enum Stage {
        FINAL_DISPLAY,
        FILTER_CONTOURS,
        CONTOURS,
        MASK,
        THRESHOLD,
        CONVERT,
        RAW_IMAGE
    }

    public Stage stageToRenderToViewport = Stage.RAW_IMAGE;
    private Stage[] stages = Stage.values();


    //Outputs
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    private Mat maskOutput = new Mat();
    private Mat convertedImage = new Mat();
    private Mat thresholdImage = new Mat();
    private double lineAngle = 0;
    private double lineAngles[] = {};
    private double lineLengths[] = {};
    private Telemetry telemetry;
    private boolean usingTelemetry;
    private int numRects;
    private Point lineCenter = null;
    private Point[] lineCenters = {};


    public BlueLineFinder(){

    }

    public BlueLineFinder(Telemetry externalTelemetry) {
        usingTelemetry = true;
        telemetry = externalTelemetry;
    }

    private void outputTelemetry(String caption, String msg){
        if (usingTelemetry){
            telemetry.addData(caption,msg);
        }
    }

    public int getNumRects(){
        return numRects;
    }

    public double[] getLineLengths(){
        return lineLengths;
    }

    @Override
    public Mat processImage(Mat in) {

            process(in);
            ArrayList<MatOfPoint> contours = filterContoursOutput();
            ArrayList<MatOfPoint> curves = new ArrayList<>();
            ArrayList<RotatedRect> boxes = new ArrayList<>();
            ArrayList<RotatedRect> boxes2 = new ArrayList<>();
            ArrayList<Mat> bestFits = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
                double epsilon = 0.05 * Imgproc.arcLength(curve, true);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(curve, approxCurve, epsilon, true);
                MatOfPoint dst = new MatOfPoint();
                approxCurve.convertTo(dst, CvType.CV_32S);
                curves.add(dst);
                RotatedRect rect = Imgproc.minAreaRect(curve);
                boxes.add(rect);
                RotatedRect rect2 = Imgproc.minAreaRect(approxCurve);
                boxes2.add(rect2);
                Mat out = new Mat();
                Imgproc.fitLine(curve, out, Imgproc.DIST_L2, 0, 0.01, 0.01);
                bestFits.add(out);
            }
            outputTelemetry("Rects found: ", ""+boxes.size());
            numRects = boxes.size();
            lineAngles = new double[numRects];
            lineCenters = new Point[numRects];
            lineLengths = new double[numRects];
            int j = 0;
            for (RotatedRect rect : boxes) {
                Point[] points = new Point[4];
                rect.points(points);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(in, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0));
                }
                double angle = rect.angle;
                if (rect.size.height > rect.size.width) {
                    angle -= 90;
                }
                lineAngles[j] = angle;
                outputTelemetry("Line angle: ", ""+angle);
                Point center = rect.center;
                lineCenters[j] = center;

                Point direction = new Point(Math.cos(angle * Math.PI / 180), Math.sin(angle * Math.PI / 180)); //find major axis directions
                double biggestAxis = Math.max(rect.size.height, rect.size.width);
                lineLengths[j] = biggestAxis;
                Point p1 = new Point(center.x + direction.x * biggestAxis, center.y + direction.y * biggestAxis);
                Point p2 = new Point(center.x - direction.x * biggestAxis, center.y - direction.y * biggestAxis);
                Imgproc.line(in, p1, p2, new Scalar(0, 255, 0));
                j++;
            }
            if (numRects > 0) {
                lineAngle = lineAngles[0];
                lineCenter = lineCenters[0];
            }
            /*
             * for (RotatedRect rect : boxes2) { Point[] points = new Point[4];
             * rect.points(points); for (int i = 0; i < 4; i++) { Imgproc.line(in,
             * points[i], points[(i + 1) % 4], new Scalar(0, 0, 255)); }
             *
             * double angle = rect.angle; if (rect.size.height > rect.size.width) { angle -=
             * 90; } System.out.println("angle 2: " + angle); Point center = rect.center;
             * Point direction = new Point(Math.cos(angle * Math.PI / 180), Math.sin(angle *
             * Math.PI / 180)); double biggestAxis = Math.max(rect.size.height,
             * rect.size.width); Point p1 = new Point(center.x + direction.x * biggestAxis,
             * center.y + direction.y * biggestAxis); Point p2 = new Point(center.x -
             * direction.x * biggestAxis, center.y - direction.y * biggestAxis);
             * Imgproc.line(in, p1, p2, new Scalar(0, 0, 255)); }
             */
            /*
             * for (Mat out : bestFits) { System.out.println("out: " + out);
             * System.out.println("out data: " + out.dump()); double rows =
             * in.size().height; double cols = in.size().width; double vx = out.get(0,
             * 0)[0]; double vy = out.get(1, 0)[0]; double x = out.get(2, 0)[0]; double y =
             * out.get(3, 0)[0]; int lefty = (int) ((-x * vy / vx) + y); int righty = (int)
             * (((cols - x) * vy / vx) + y); Imgproc.line(in, new Point(cols - 1, righty),
             * new Point(0, lefty), new Scalar(0, 0, 0), 2); }
             */
            Mat out = new Mat();
            maskOutput.copyTo(out);
            Scalar color = new Scalar(0,0,255);
            if (getMaskedImageColorSample() != null){
                color = getMaskedImageColorSample();
            }
            Imgproc.line(out,new Point(100,100),new Point(100,125), color);
            Imgproc.line(out,new Point(100,125),new Point(125,125), color);
            Imgproc.line(out,new Point(125,125),new Point(125,100), color);
            Imgproc.line(out,new Point(125,100),new Point(100,100), color);


            if (stageToRenderToViewport == Stage.RAW_IMAGE)
                out = in;

            if (stageToRenderToViewport == Stage.CONVERT)
                out = convertedImage;
            if (stageToRenderToViewport == Stage.THRESHOLD)
                out = thresholdImage;
            if (stageToRenderToViewport == Stage.CONTOURS || stageToRenderToViewport == Stage.FILTER_CONTOURS) {
                out = maskOutput();
                ArrayList<MatOfPoint> drawingContours = findContoursOutput;
                Scalar contourColor = new Scalar(255,0,0);
                if (stageToRenderToViewport == Stage.FILTER_CONTOURS) {
                    drawingContours = filterContoursOutput;
                    contourColor = new Scalar(255,255,0);
                }
                for (int i = 0; i < drawingContours.size(); i++) {
                    Imgproc.drawContours(out,drawingContours,i,contourColor);
                }
            }
            numRects = (int)out.size().width;

            return out;


    }




    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0) {
        // Step HSV_Threshold0:
        Mat hsvThresholdInput = source0;


        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = true;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
        double filterContoursMinArea = 2000.0;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 0.0;
        double filterContoursMaxWidth = 1000.0;
        double filterContoursMinHeight = 0.0;
        double filterContoursMaxHeight = 1000.0;
        double[] filterContoursSolidity = {0.0, 100.0};
        double filterContoursMaxVertices = 2.147483647E9;
        double filterContoursMinVertices = 0.0;
        double filterContoursMinRatio = 0.0;
        double filterContoursMaxRatio = 1000.0;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        // Step Mask0:
        Mat maskInput = source0;
        Mat maskMask = hsvThresholdOutput;
        mask(maskInput, maskMask, maskOutput);

    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }

    /**
     * This method is a generated getter for the output of a Filter_Contours.
     * @return ArrayList<MatOfPoint> output from Filter_Contours.
     */
    public ArrayList<MatOfPoint> filterContoursOutput() {
        return filterContoursOutput;
    }

    /**
     * This method is a generated getter for the output of a Mask.
     * @return Mat output from Mask.
     */
    public Mat maskOutput() {
        return maskOutput;
    }

    /**
     * This method returns the angle of the line detected beneath the robot
     * @return double angle
     */
    public double getLineAngle() {
        return lineAngle;
    }

    public double[] getLineAngles() {
        return lineAngles;
    }

    /**
     * This method returns the position of the center of the line seen by the viewport
     * @return double angle
     */
    public Point getLineCenter() {
        return lineCenter;
    }

    public Point[] getLineCenters() {
        return lineCenters;
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
        //System.out.println(input.type());

        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Mat copy = new Mat();
        out.copyTo(copy);
        convertedImage = copy;
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
        Mat copy2 = new Mat();
        out.copyTo(copy2);
        thresholdImage = copy2;
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
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
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


    public Scalar getMaskedImageColorSample() {
        int startx = 100;
        int endx = 125;
        int starty = 100;
        int endy = 125;
        for (int i = startx; i < endx; i++) {
            for (int j = starty; j < endy; j++) {
                double[] color = maskOutput.get(i,j);
                if (color[0] + color[1] + color[2] > 30){
                    return new Scalar(color);
                }
            }
        }
        return null;
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
        return "Blue Line finder, displayed stage: " + stageToRenderToViewport.name();
    }
}


