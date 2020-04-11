package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public abstract class BetterOpenCVPipeline extends OpenCvPipeline {
    private DisplayElement[] myElements = {};

    public void updateElements(DisplayElement[] inElements){
        myElements = inElements;
    }
    public void updateElements(ArrayList<DisplayElement> inElements){

        myElements = new DisplayElement[inElements.size()];
        for (int i = 0; i < inElements.size(); i++){
            myElements[i] = inElements.get(i);
        }
    }

    public abstract Mat processImage(Mat in);

    @Override
    public Mat processFrame(Mat in) {
        Mat out = new Mat();
        in.copyTo(out);

        processImage(out);

        for (DisplayElement element : myElements)
        {
            drawElement(out,element);
        }

        return out;
    }

    private void drawElement(Mat mat, DisplayElement element){
        switch (element.myType){
            case POINT:
                Imgproc.drawMarker(mat,element.myPoints[0],element.myColor,0,element.mySize);
                break;
            case VECTOR:
                Imgproc.arrowedLine(mat,element.myPoints[0],element.myPoints[1],element.myColor);
                break;
            case SHAPE:
                for (int i = 0; i < element.myPoints.length; i++){
                    Imgproc.line(mat,element.myPoints[i],element.myPoints[(i+1)%element.myPoints.length],element.myColor);
                }
        }
    }



}
