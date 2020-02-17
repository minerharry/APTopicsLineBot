package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JointPipeline extends OpenCvPipeline {

    private Point[] displayPoints = {};
    private OpenCvPipeline[] myPipelines;
    private int viewedPipeline;
    public int getViewedPipelineNum(){
        return viewedPipeline;
    }
    private boolean cyclePipelines;
    private boolean drawDisplayPoints = false;
    private Scalar displayColor = new Scalar(0,0,255);

    public void setDisplayPoints(Point[] points){
        drawDisplayPoints = true;
        displayPoints = points;
    }

    public void setDisplayPoints(Point[] points, Scalar color){
        drawDisplayPoints = true;
        displayPoints = points;
        displayColor = color;
    }

    public JointPipeline(OpenCvPipeline[] pipelines) {
        myPipelines = pipelines;
        cyclePipelines = true;
    }
    public JointPipeline(OpenCvPipeline[] pipelines, int fixedViewPipeline){
        myPipelines = pipelines;
        cyclePipelines = true;
        viewedPipeline = fixedViewPipeline;
        cyclePipelines = false;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat copyMat = new Mat();
        Mat output = new Mat();
        for (int i = 0; i < myPipelines.length; i++){
            input.copyTo(copyMat);
            OpenCvPipeline pipeline = getPipeline(i);
            if (viewedPipeline == i)
                output = pipeline.processFrame(input);
            else
                pipeline.processFrame(input);
        }

        if (drawDisplayPoints){
            for(Point point : displayPoints){
                Imgproc.drawMarker(output,point,displayColor);
            }
        }

        return output;

    }

    public OpenCvPipeline getViewedPipeline(){
        return getPipeline(viewedPipeline);
    }

    public OpenCvPipeline getPipeline(int i){
        return myPipelines[i];
    }

    @Override
    public void onViewportTapped(){
        if (!cyclePipelines){
            OpenCvPipeline vPipeline = getViewedPipeline();
            vPipeline.onViewportTapped();
        }
        else {
            viewedPipeline = (viewedPipeline >= myPipelines.length ? 0 : viewedPipeline + 1);
        }
    }



}
