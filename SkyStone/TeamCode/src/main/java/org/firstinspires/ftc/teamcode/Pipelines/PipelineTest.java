package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineTest extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat in){
        Mat out = new Mat();
        Imgproc.cvtColor(in,out,Imgproc.COLOR_BGR2HSV);
        double[] hue = {45,70};
        double[] all = {0,255.0};
        Core.inRange(out,new Scalar(hue[0],all[0],all[0]),new Scalar(hue[1],all[1],all[1]),out);
        return out;

    }
}
