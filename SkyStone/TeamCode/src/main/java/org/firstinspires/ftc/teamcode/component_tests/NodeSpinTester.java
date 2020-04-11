package org.firstinspires.ftc.teamcode.component_tests;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AngleUtils;
import org.firstinspires.ftc.teamcode.Pipelines.BlueLineFinder;
import org.firstinspires.ftc.teamcode.Pipelines.GreenNodeFinder;
import org.firstinspires.ftc.teamcode.Pipelines.JointPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="node_spinner_tester")
public class NodeSpinTester extends OpMode {

    private GreenNodeFinder nodePipeline;
    private BlueLineFinder linePipeline;
    private JointPipeline jointPipeline;

    private OpenCvWebcam webcam;
    private double targetAngle;


    private Orientation lastOrientation = new Orientation();
    private AxesOrder axes = AxesOrder.ZYX;

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor[] motors;

    private BNO055IMU imu;

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

    private boolean auto = false;
    private boolean last_a = false;
    private final double movement_speed = 0.2;
    private final double auto_slowdown = 1.2;
    private final double steer_coeff = 0.4;
    private final double dead_zone_right = 0.3;
    private final double dead_zone_left = 0.3;
    private final double max_turn_angle = 2*Math.PI/5;
    private final double min_reverse_angle = 3*Math.PI/5;


    @Override
    public void init() {


        imu = hardwareMap.get(BNO055IMU.class,"imu");



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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu.initialize(parameters);


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
        nodePipeline = new GreenNodeFinder();
        linePipeline = new BlueLineFinder();
        OpenCvPipeline[] pipelines = {nodePipeline,linePipeline};
        jointPipeline = new JointPipeline(pipelines);
        webcam.setPipeline(jointPipeline);

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

        telemetry.addData("Pipeline Viewed: ", jointPipeline.getViewedPipelineNum());
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Current Gyro angle: ",angles.firstAngle);

        double[] robotCenter = {nodePipeline.getFindNodeImageOutput().width()/2, nodePipeline.getFindNodeImageOutput().height()/2};

        double targetAngle = (Math.PI*2 - AngleUtils.angleBetweenPoints(robotCenter, nodePipeline.getFindNodeCenter()))%(Math.PI*2) - Math.PI;
        telemetry.addData("Raw Angle: ", AngleUtils.angleBetweenPoints(robotCenter, nodePipeline.getFindNodeCenter()));
        telemetry.addData("Target Angle: ", targetAngle);

        if (gamepad1.a && !last_a){
            auto = !auto;
        }
        last_a = gamepad1.a;

        if (auto){
            // adjust relative speed based on heading error.
            double distance = Math.sqrt(Math.pow(robotCenter[0]- nodePipeline.getFindNodeCenter().x,2) + Math.pow(robotCenter[1]- nodePipeline.getFindNodeCenter().y,2));
            telemetry.addData("Distance to target: ", distance);
            if (distance > 20) {

                double speed = (Math.abs(targetAngle) < max_turn_angle ? movement_speed : (Math.abs(targetAngle) > min_reverse_angle ? -movement_speed : 0));
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




        double[] lineAngles = linePipeline.getLineAngles();
        telemetry.addData("Line angles: ", arrayToString(lineAngles));
        double[] shiftedAngles = new double[lineAngles.length];
        double[] shiftedAngles2 = new double[lineAngles.length];

        for (int i = 0; i < linePipeline.getLineCenters().length; i++){
            Point centerPoint = linePipeline.getLineCenters()[i];
            int sign = (centerPoint != null && centerPoint.y<nodePipeline.getFindNodeCenter().y ? -1 : 0);
            shiftedAngles[i] = 180*sign+ lineAngles[i] - angles.firstAngle*180/Math.PI;
            shiftedAngles2[i] =  lineAngles[i] - angles.firstAngle*180/Math.PI;

        }
        telemetry.addData("Line shifted angles smart: ", arrayToString(shiftedAngles));
        telemetry.addData("Line shifted angles: ", arrayToString(shiftedAngles));

        telemetry.addData("Node center: ", nodePipeline.getFindNodeCenter().toString());
        //telemetry.addData("Node radius: ", pipeline.getFindNodeRadius() + "");
        //telemetry.addData("Node area: ", pipeline.getFindNodeArea() + "");
    }

    public String arrayToString(double[] array){
        String result = " ";
        for (Object obj : array){
            result += obj + ",";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }




}
