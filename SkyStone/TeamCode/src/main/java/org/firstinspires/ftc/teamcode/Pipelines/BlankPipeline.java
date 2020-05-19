package org.firstinspires.ftc.teamcode.Pipelines;

import android.provider.ContactsContract;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlankPipeline extends BetterOpenCVPipeline {

    private boolean useInputMat = true;
    private Scalar backgroundColor = new Scalar(255,255,255);

    public BlankPipeline(boolean useInputMat){
        this.useInputMat = useInputMat;
    }

    public BlankPipeline(Scalar backgroundColor){
        useInputMat = false;
        this.backgroundColor = backgroundColor;
    }


    @Override
    public Mat processImage(Mat in) {
        Mat out = new Mat();
        in.copyTo(out);
        if (!useInputMat) {
            out.setTo(backgroundColor);
        }
        return out;
    }

    @Override
    public String getDisplayedStageName() {
        return "Blank pipeline, " + (useInputMat ? "basic input rendered" : backgroundColor + " background rendered");
    }

}
