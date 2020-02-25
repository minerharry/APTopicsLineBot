package org.firstinspires.ftc.teamcode.vision_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AngleSystem;
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

    private boolean speed_toggle = true;
    private boolean last_bumper = false;
    private boolean auto = false;
    private boolean last_a = false;
    private final double auto_movement_speed = 0.5;
    private final double line_speed = 0.35;
    private final double line_steer_coeff = 0.5; //how quickly it goes from min steer to max steer based on angle
    private final double line_max_steer = 0.8; //maximum steer that will be applied
    private final double line_turn_angle = 1*Math.PI/4;
    private final double line_reverse_angle = 4*Math.PI/5;
    private final double pixels_per_cm = 1;
    private final double cm_ahead_prediction = 250; //incorrect for now - pixels_per_inch should be measured before a real unit is used


    private final double manual_movement_speed = 1;
    private final double dead_zone_right = 0.3;
    private final double dead_zone_left = 0.3;

    private final AngleSystem OpenCV_angles = new AngleSystem(0, false,true);
    private final AngleSystem robot_angles = new AngleSystem(Math.PI/2,false);


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
        double[] targetPoint = {0,0};
        if (pipeline.getLineCenter() != null) {
            double[] lineCenter = {pipeline.getLineCenter().x, pipeline.maskOutput().height() - pipeline.getLineCenter().y};
            targetPoint = AngleUtils.projectPoint(OpenCV_angles.toGlobal(pipeline.getLineAngle()), cm_ahead_prediction * pixels_per_cm, lineCenter);
        }


        telemetry.addData("Stage: ", pipeline.stageToRenderToViewport);

        telemetry.addData("Target Point: ", targetPoint[0] + ", " + targetPoint[1]);

        pipeline.addDisplayPoint(new Point(targetPoint[0],pipeline.maskOutput().height() - targetPoint[1]));

        double targetAngle = (AngleUtils.angleBetweenPoints(robotCenter,targetPoint) + 3*Math.PI)%(Math.PI*2) - Math.PI;
        telemetry.addData("Target Angle: ", targetAngle);

        double manual_slowdown = (speed_toggle ? 1 : 0)*(gamepad1.right_trigger < 0.1 ? (gamepad1.left_trigger < 0.1 ? 1 : 0.5) : 2);
        if (!last_bumper && gamepad1.right_bumper){
            speed_toggle = !speed_toggle;
        }
        last_bumper = gamepad1.right_bumper;

        if (gamepad1.a && !last_a){
            auto = !auto;
        }
        last_a = gamepad1.a;

        if (auto){
            // adjust relative speed based on heading error.
            double distance = Math.sqrt(Math.pow(robotCenter[0]-targetPoint[0],2) + Math.pow(robotCenter[1]-targetPoint[1],2));
            telemetry.addData("Distance to target: ", distance);
            if (distance > 20) {

                double steer = Range.clip(targetAngle*line_steer_coeff,-1,1)* line_max_steer;
                double maxSpeed = 1;
                double speed = (Math.abs(targetAngle) < line_turn_angle ? line_speed : (Math.abs(targetAngle) > line_reverse_angle ? -line_speed : 0.05*-line_speed));


                double leftSpeed = speed  + steer;
                double rightSpeed = speed - steer;
                if (leftSpeed != 0 || rightSpeed != 0){
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > maxSpeed) {
                        leftSpeed = leftSpeed/(max)*maxSpeed;
                        rightSpeed = rightSpeed/(max)*maxSpeed;
                    }
                }
                double leftPower = auto_movement_speed*leftSpeed*manual_slowdown;
                double rightPower = auto_movement_speed*rightSpeed*manual_slowdown;
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
            FR.setPower(right_input*manual_movement_speed*manual_slowdown);
            BR.setPower(right_input*manual_movement_speed*manual_slowdown);
            FL.setPower(left_input*manual_movement_speed*manual_slowdown);
            BL.setPower(left_input*manual_movement_speed*manual_slowdown);
        }






        //telemetry.addData("Node radius: ", pipeline.getFindNodeRadius() + "");
        //telemetry.addData("Node area: ", pipeline.getFindNodeArea() + "");
    };

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
