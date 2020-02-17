package org.firstinspires.ftc.teamcode.vision_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AngleUtils;
import org.firstinspires.ftc.teamcode.Pipelines.BlueLineFinder;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Line_approach_tester")
public class LineApproachTester extends OpMode {

    enum motorNames {
        FrontLeft("front_left"),
        FrontRight("front_right"),
        BackLeft("back_left"),
        BackRight("back_right");
        String myName;
        motorNames(String name) {
            myName = name;
        }
        String getName(){
            return myName;
        }
    }

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor[] motors;

    private boolean auto = false;
    private boolean last_a = false;
    private final double movement_speed = 0.2;
    private final double auto_slowdown = 1.2;
    private final double steer_coeff = 0.4;
    private final double dead_zone_right = 0.3;
    private final double dead_zone_left = 0.3;
    private final double max_turn_angle = 2*Math.PI/5;
    private final double pixels_per_cm = 1;
    private final double cm_ahead_prediction = 250; //incorrect for now - pixels_per_inch should be measured before a real unit is used



    private OpenCvWebcam webcam;
    private BlueLineFinder pipeline;

    @Override
    public void init() {

        FL = hardwareMap.get(DcMotor.class, motorNames.FrontLeft.getName());
        FR = hardwareMap.get(DcMotor.class, motorNames.FrontRight.getName());
        BL = hardwareMap.get(DcMotor.class, motorNames.BackLeft.getName());
        BR = hardwareMap.get(DcMotor.class, motorNames.BackRight.getName());
        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new BlueLineFinder();
        webcam.setPipeline(pipeline);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void loop() {

        double[] robotCenter = {pipeline.maskOutput().width()/2,pipeline.maskOutput().height()/2};


        double[] testProjection = AngleUtils.projectPoint(Math.PI/4,Math.sqrt(2)*10);
        telemetry.addData("test point: ", testProjection[0] + ", " + testProjection[1]);

        double[] targetPoint = AngleUtils.projectPoint(pipeline.getLineAngle()*Math.PI/180,cm_ahead_prediction*pixels_per_cm,AngleUtils.getPointPos(pipeline.getLineCenter()));


        if (pipeline.getLineCenter() != null)
            telemetry.addData("Line center: ",pipeline.getLineCenter().x + ", " + pipeline.getLineCenter().y);

        telemetry.addData("Stage: ", pipeline.stageToRenderToViewport);

        telemetry.addData("Target Point: ", targetPoint[0] + ", " + targetPoint[1]);

        pipeline.addDisplayPoint(new Point(targetPoint[0],targetPoint[1]));

        double targetAngle = (Math.PI*2 - AngleUtils.angleBetweenPoints(robotCenter,targetPoint))%(Math.PI*2) - Math.PI;
        telemetry.addData("Target Angle: ", targetAngle);

        if (gamepad1.a && !last_a){
            auto = !auto;
        }
        last_a = gamepad1.a;

        if (auto){
            // adjust relative speed based on heading error.
            double distance = Math.sqrt(Math.pow(robotCenter[0]-targetPoint[0],2) + Math.pow(robotCenter[1]-targetPoint[1],2));
            telemetry.addData("Distance to target: ", distance);
            if (distance > 20) {

                double speed = (Math.abs(targetAngle) < max_turn_angle ? movement_speed : 0);
                double steer = getSteer(targetAngle, steer_coeff);


                double idealSpeed = Math.min(1.0,distance/100)*auto_slowdown;

                double leftSpeed = speed + steer;
                double rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > idealSpeed) {
                    leftSpeed = leftSpeed/(max)*idealSpeed;
                    rightSpeed = rightSpeed/(max)*idealSpeed;
                }
                double leftPower = -leftSpeed;
                double rightPower = -rightSpeed;
                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);
            }
            else {
                for (DcMotor motor :motors) {
                    motor.setPower(0.0);
                }
            }
        } else {
            telemetry.addData("Left power: ",gamepad1.left_stick_y);
            telemetry.addData("Right power: ",gamepad1.right_stick_y);
            double right_input = (Math.abs(gamepad1.right_stick_y) > dead_zone_right ? gamepad1.right_stick_y : 0);
            double left_input = (Math.abs(gamepad1.left_stick_y) > dead_zone_left ? gamepad1.left_stick_y : 0);
            FR.setPower(right_input*movement_speed);
            BR.setPower(right_input*movement_speed);
            FL.setPower(left_input*movement_speed);
            BL.setPower(left_input*movement_speed);
        }






        //telemetry.addData("Node radius: ", pipeline.getFindNodeRadius() + "");
        //telemetry.addData("Node area: ", pipeline.getFindNodeArea() + "");
    };

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
