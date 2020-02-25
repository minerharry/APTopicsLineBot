package org.firstinspires.ftc.teamcode.vision_tests;

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
import org.firstinspires.ftc.teamcode.AngleSystem;
import org.firstinspires.ftc.teamcode.Pipelines.BlueLineFinder;
import org.firstinspires.ftc.teamcode.Pipelines.GreenNodeFinder;
import org.firstinspires.ftc.teamcode.Pipelines.JointPipeline;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Random;

@TeleOp(name="navTest")
public class NodeNavigateTester extends OpMode {

    //Motor stuff
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

    //input stuff
    private boolean last_a = false;
    private boolean speed_toggle = true;
    private boolean last_bumper = false;
    private final double manual_movement_speed = 1;
    private final double dead_zone_right = 0.3;
    private final double dead_zone_left = 0.3;

    //nav constants
    private final double auto_movement_speed = 0.5;

    private final double line_speed = 0.35;
    private final double line_steer_coeff = 0.5; //how quickly it goes from min steer to max steer based on angle
    private final double line_max_steer = 0.8; //maximum steer that will be applied
    private final double line_turn_angle = 1*Math.PI/4;
    private final double line_reverse_angle = 4*Math.PI/5;

    private final double line_angle_error_threshold = Math.PI/10;

    private final double node_speed = 0.25;
    private final double node_steer_coeff = 0.5; //how quickly it goes from min steer to max steer based on angle
    private final double node_max_steer = 0.6; //maximum steer that will be applied
    private final double node_forward_angle = 2*Math.PI/5;
    private final double node_reverse_angle = 3*Math.PI/5;
    private final double node_distance_threshold = 15; //how close it must get to the node to be considered 'on' the node

    private final double found_node_perpendicular_threshold = 50; //in pixels
    private final double found_node_parallel_threshold = 20;

    private final double pixels_per_cm = 1;
    private final double cm_ahead_prediction = 250; //incorrect for now - pixels_per_inch should be measured before a real unit is used

    //angle stuff
    private final AngleSystem OpenCV_angles = new AngleSystem(0, false,true);
    private final AngleSystem robot_angles = new AngleSystem(Math.PI/2,false);



    //Gyro stuff
    private BNO055IMU imu;
    private Orientation currentOrientation = new Orientation();
    private Orientation lastOrientation = new Orientation();
    private AxesOrder axes = AxesOrder.ZYX;

    //Opencv stuff
    private GreenNodeFinder nodePipeline;
    private BlueLineFinder linePipeline;
    private JointPipeline jointPipeline;
    private OpenCvWebcam webcam;

    private Point[] lineCenters;
    private Point nodeCenter;
    private double[] lineAngles;


    //auto stuff
    enum AutoMode {
        FOLLOW_LINE,
        ANGLES_FROM_NODE
    }
    private boolean auto = false;
    private AutoMode currentMode = AutoMode.FOLLOW_LINE;
    private double lineApproxAngle = 0;
    private int nodeSeenCounter = 0;
    private static final int nodeSeenThreshold = 4;

    private int nodeNotSeenCounter = 0;
    private static final int nodeNodeSeenThreshold = 6;


    @Override
    public void init(){
        //init motors
        FL = hardwareMap.get(DcMotor.class, motorNames.FrontLeft.getName());
        FR = hardwareMap.get(DcMotor.class, motorNames.FrontRight.getName());
        BL = hardwareMap.get(DcMotor.class, motorNames.BackLeft.getName());
        BR = hardwareMap.get(DcMotor.class, motorNames.BackRight.getName());
        DcMotor[] tempMotors = {FL, FR, BL, BR};
        motors = tempMotors;

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //init gyro
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu.initialize(parameters);

        //init camerastuff
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

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


    }

    @Override
    public void loop(){
        lineAngles = linePipeline.getLineAngles();
        lineCenters = linePipeline.getLineCenters();
        nodeCenter = nodePipeline.getFindNodeCenter();



        //get gyro angle
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Current Gyro angle: ",currentOrientation.firstAngle);

        double[] targetPoint = getTargetPoint();
        double[] robotCenter = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2}; //would need to invert y, but exactly halfway in the image, so not needed
        double targetAngle =(robot_angles.fromGlobal(AngleUtils.angleBetweenPoints(robotCenter,targetPoint)) + 3*Math.PI)%(Math.PI*2) - Math.PI;//angleBetweenPoints uses tan from the right, convert to tan from the front
        double targetDistance = AngleUtils.distance(targetPoint,robotCenter);

        //display approxLineAngle and chosen line
        double opencvAngle = (robot_angles.toGlobal(lineApproxAngle + currentOrientation.firstAngle));
        Point p1 = new Point(robotCenter);
        Point p2 = new Point(AngleUtils.projectPoint(opencvAngle,20,robotCenter));
        p2.y = linePipeline.maskOutput().height()- p2.y;
        Point p3 = new Point();
        if (getFollowedLineIndex() >= 0 && lineCenters.length > 0){
            p3 = lineCenters[getFollowedLineIndex()];
        }

        Point[] displayPoints = {p1,p2,p3, new Point(targetPoint[0],linePipeline.maskOutput().height()-targetPoint[1])};
        jointPipeline.setDisplayPoints(displayPoints,new Scalar(255,0,0));


        telemetry.addData("Test distance", AngleUtils.shortestAngleBetween(1.5707,1.588));
        telemetry.addData("Line Angles: ", arrayToString(lineAngles));
        telemetry.addData("Line Centers: ", arrayToString(lineCenters));

        telemetry.addData("Target Pos: ", arrayToString(targetPoint));
        telemetry.addData("Target angle: ", targetAngle);
        telemetry.addData("Target distance: ", targetDistance);

        double[] shiftedAngles = getShiftedLineAngles(nodeCenter != null);
        double[] realAngles = new double[shiftedAngles.length];
        for (int i = 0; i < realAngles.length; i++)
        {
            realAngles[i] = robot_angles.fromGlobal(shiftedAngles[i]);
        }
        telemetry.addData("robot angled lines", arrayToString(realAngles));
        telemetry.addData("robot line approx Angle", (lineApproxAngle));

        telemetry.addData("global angled lines", arrayToString(shiftedAngles));
        telemetry.addData("global line approx Angle", robot_angles.toGlobal(lineApproxAngle));


        double[] test1 = {2,-0.5};
        double targetTest = gamepad1.left_stick_y;
        telemetry.addData("Closest Angle", test1[AngleUtils.nearestAngle(targetTest,test1)]);



        telemetry.addData("Trigger values","Left: " + gamepad1.left_trigger + ", Right: " + gamepad1.right_trigger);
        double manual_slowdown = (speed_toggle ? 1 : 0)*(gamepad1.right_trigger < 0.1 ? (gamepad1.left_trigger < 0.1 ? 1 : 0.5) : 2);
        if (!last_bumper && gamepad1.right_bumper){
            speed_toggle = !speed_toggle;
        }
        last_bumper = gamepad1.right_bumper;



        //check auto switching
        if (gamepad1.a && !last_a){
            auto = !auto;
        }
        last_a = gamepad1.a;


        //do movement
        if (auto){
            telemetry.addData("Auto: ", "Enabled");
            telemetry.addData("Auto Mode: ", currentMode.name());
            double maxSpeed = 0; //from -1 - 1; all output values relative to maxSpeed being the max. Scaled down to auto speeds later
            double steer = 0;
            double speed = 0;
            switch (currentMode){
                case FOLLOW_LINE:
                    steer = Range.clip(targetAngle*line_steer_coeff,-1,1)* line_max_steer;
                    maxSpeed = 1;
                    speed = (Math.abs(targetAngle) < line_turn_angle ? line_speed : (Math.abs(targetAngle) > line_reverse_angle ? -line_speed : 0.05*-line_speed));
                    break;
                case ANGLES_FROM_NODE:
                    maxSpeed = Math.min(targetDistance/40,1);
                    speed = (Math.abs(targetAngle) < node_forward_angle ? node_speed : 0);
                    if (Math.abs(targetAngle) > node_reverse_angle) {
                        speed = -node_speed;
                        targetAngle = 180-targetAngle;
                        targetAngle = (targetAngle + Math.PI*3) % (Math.PI*2) - Math.PI;
                    }
                    steer = Range.clip(targetAngle*node_steer_coeff,-1,1)* node_max_steer;
                    if (targetDistance < node_distance_threshold) {
                        speed = 0;
                        maxSpeed = 0;
                        steer = 0;
                    }
                    break;
            }
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
        } else {
            manualInput(manual_slowdown);
        }


        //auto logic stuff
        if (auto) {
            if (currentMode == AutoMode.FOLLOW_LINE) {
                if (lineAngles != null && lineAngles.length > 0) {
                    if (nodeCenter != null) {
                        double[] nodePos = AngleUtils.getPointPos(nodeCenter);
                        nodePos[1] = nodePipeline.getFindNodeImageOutput().height() - nodePos[1];
                        int targetIndex = getFollowedLineIndex(false);
                        double[] linePoint = AngleUtils.getPointPos(lineCenters[targetIndex]);
                        linePoint[1] = linePipeline.maskOutput().height() - linePoint[1];
                        if (Math.abs(getFollowedLineError(false)) > line_angle_error_threshold) {
                            linePoint = robotCenter;
                            telemetry.addData("error","prone");
                        }
                        double perpDist = AngleUtils.perpendicularDistance(linePoint, robot_angles.toGlobal(lineApproxAngle), nodePos);
                        double parDist = AngleUtils.parallelDistance(linePoint, robot_angles.toGlobal(lineApproxAngle), nodePos);
                        telemetry.addData("Perp Inputs",arrayToString(linePoint) + ", " + robot_angles.toGlobal(lineApproxAngle) + ", " + arrayToString(nodePos));
                        telemetry.addData("ParDist",parDist);
                        telemetry.addData("PerpDist",perpDist);
                        if (parDist > found_node_parallel_threshold && perpDist < found_node_perpendicular_threshold) {
                            nodeSeenCounter++;
                        }else{
                            nodeSeenCounter = Range.clip(nodeSeenCounter - 2, 0, 100);
                        }
                        if (nodeSeenCounter > nodeSeenThreshold) {
                            nodeSeenCounter = 0;
                            currentMode = AutoMode.ANGLES_FROM_NODE;
                        }
                    } else
                        nodeSeenCounter = Range.clip(nodeSeenCounter - 2, 0, 100);
                    if (Math.abs(getFollowedLineError()) < line_angle_error_threshold) {
                        lineApproxAngle = robot_angles.fromGlobal(getFollowedLineAngle());
                        telemetry.addData("Updated target angle",true);
                    }
                    else
                        telemetry.addData("Updated target angle",false);
                }
            } else if (currentMode == AutoMode.ANGLES_FROM_NODE) {
                if (targetDistance < node_distance_threshold) {
                    if (lineAngles != null && lineAngles.length > 0) {
                        double[] angles = getShiftedLineAngles(true);
                        lineApproxAngle = robot_angles.fromGlobal(angles[(int) (Math.random() * angles.length)]);
                        currentMode = AutoMode.FOLLOW_LINE;
                    }
                }
            }
        }


        try{
            boolean[] bools = {true,false};
            for (int i = 0; i < 2; i++) {
                telemetry.addData("UnNodedAngles" + bools[i], arrayToString(getShiftedLineAngles(bools[i])));
                telemetry.addData("UnNodedIndex" + bools[i], getFollowedLineIndex(bools[i]));
                telemetry.addData("UnNodedAngle" + bools[i], getFollowedLineAngle(bools[i]));
                telemetry.addData("Error" + bools[i], getFollowedLineError(bools[i]));
            }
        } catch (Exception e){
            telemetry.addData("Error","errored");
        }

        //set last gyro angle
        lastOrientation = currentOrientation;
    }



    public void manualInput(double scale){
        telemetry.addData("Left power: ",gamepad1.left_stick_y);
        telemetry.addData("Right power: ",gamepad1.right_stick_y);
        double right_input = (Math.abs(gamepad1.right_stick_y) > dead_zone_right ? gamepad1.right_stick_y : 0);
        double left_input = (Math.abs(gamepad1.left_stick_y) > dead_zone_left ? gamepad1.left_stick_y : 0);
        FR.setPower(-right_input*manual_movement_speed*scale);
        BR.setPower(-right_input*manual_movement_speed*scale);
        FL.setPower(-left_input*manual_movement_speed*scale);
        BL.setPower(-left_input*manual_movement_speed*scale);
    }



    private int getFollowedLineIndex(){
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        double[] shiftedAngles;
        if (nodeCenter != null)
        {
            double[] nodePos = {nodeCenter.x,linePipeline.maskOutput().height()-nodeCenter.y};
            double[] robotPos = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};
            if (AngleUtils.parallelDistance(robotPos,targetAngle,nodePos) < 0) {
                targetAngle += Math.PI;
            }
        }
        if (nodeCenter != null) { //get exact directions based on node
            shiftedAngles = getShiftedLineAngles(true);
        }
        else{ //check line angles in both directions, actual angle sorted out with getFollowedLineAngle
            shiftedAngles = getShiftedLineAngles(false);
            return AngleUtils.nearestAngle(targetAngle,shiftedAngles)/2;
        }
        return AngleUtils.nearestAngle(targetAngle,shiftedAngles);
    }

    private int getFollowedLineIndex(boolean nodeOverride){
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        if (nodeOverride)
        {
            double[] nodePos = {nodeCenter.x,linePipeline.maskOutput().height()-nodeCenter.y};
            double[] robotPos = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};
            if (AngleUtils.parallelDistance(robotPos,targetAngle,nodePos) < 0) {
                targetAngle += Math.PI;
            }
        }
        double[] shiftedAngles;
        if (nodeOverride) { //get exact directions based on node
            shiftedAngles = getShiftedLineAngles(true);
        }
        else{ //check line angles in both directions, actual angle sorted out with getFollowedLineAngle
            shiftedAngles = getShiftedLineAngles(false);
            return AngleUtils.nearestAngle(targetAngle,shiftedAngles)/2;
        }
        return AngleUtils.nearestAngle(targetAngle,shiftedAngles);
    }

    //if not useNode, will return a list that is twice as long, with each pair 2i and 2i+1 being 180 degrees of each other
    private double[] getShiftedLineAngles(boolean useNode){
        double[] shiftedAngles = new double[lineAngles.length];
        if (useNode) {
            double[] nodePoint = {nodeCenter.x,linePipeline.maskOutput().height() - nodeCenter.y};
            for (int i = 0; i < lineCenters.length; i++) {
                double[] centerPoint = {lineCenters[i].x, linePipeline.maskOutput().height() - lineCenters[i].y};
                double angle = OpenCV_angles.toGlobal(lineAngles[i]);
                boolean sign = AngleUtils.parallelDistance(nodePoint,angle,centerPoint) <= 0;
                telemetry.addData("Parallel line check ", AngleUtils.parallelDistance(nodePoint,angle,centerPoint));
                shiftedAngles[i] = (Math.PI * (sign ? 1 : 0) + angle + currentOrientation.firstAngle + Math.PI*6) % (Math.PI*2);
            }
        }
        else {
            shiftedAngles = new double[lineAngles.length*2];
            for (int i = 0; i < lineAngles.length*2; i++)
            {
                shiftedAngles[i] = (OpenCV_angles.toGlobal(lineAngles[i/2] + (i%2 == 0 ? 180 : 0)) + currentOrientation.firstAngle + Math.PI*6) % (Math.PI*2);
            }
        }
        return  shiftedAngles;
    }

    //returns in global angles
    private double getFollowedLineAngle() {
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        double[] shiftedAngles = getShiftedLineAngles(nodeCenter != null);
        if (nodeCenter != null)
        {
            double[] nodePos = {nodeCenter.x,linePipeline.maskOutput().height()-nodeCenter.y};
            double[] robotPos = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};
            if (AngleUtils.parallelDistance(robotPos,targetAngle,nodePos) < 0) {
                targetAngle += Math.PI;
            }
        }
        return shiftedAngles[AngleUtils.nearestAngle(targetAngle,shiftedAngles)];
    }

    private double getFollowedLineAngle(boolean nodeOverride) {
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        double[] shiftedAngles = getShiftedLineAngles(nodeOverride);
        if (nodeOverride)
        {
            double[] nodePos = {nodeCenter.x,linePipeline.maskOutput().height()-nodeCenter.y};
            double[] robotPos = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};
            if (AngleUtils.parallelDistance(robotPos,targetAngle,nodePos) < 0) {
                targetAngle += Math.PI;
            }
        }
        return shiftedAngles[AngleUtils.nearestAngle(targetAngle,shiftedAngles)];
    }

    private double getFollowedLineError() {
        double closestAngle = getFollowedLineAngle();
        return AngleUtils.shortestAngleBetween(closestAngle,robot_angles.toGlobal(lineApproxAngle));
    }
    private double getFollowedLineError(boolean nodeOverride) {
        double closestAngle = getFollowedLineAngle(nodeOverride);
        return AngleUtils.shortestAngleBetween(closestAngle,robot_angles.toGlobal(lineApproxAngle));
    }

    //returns in realworld pixel space (shifted from opencv's y, but 1 unit per pixel still)
    public double[] getTargetPoint(){
        double[] result = new double[2];
        switch (currentMode) {
            case FOLLOW_LINE:
                double lineAngle = robot_angles.toGlobal(lineApproxAngle) - currentOrientation.firstAngle;
                double[] linePoint =  {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};;
                if (lineCenters != null && lineCenters.length > 0) {
                    if (Math.abs(getFollowedLineError()) < line_angle_error_threshold) {
                        int targetIndex = getFollowedLineIndex();
                        Point center = lineCenters[targetIndex];
                        linePoint = AngleUtils.getPointPos(center);
                        linePoint[1] = linePipeline.maskOutput().height() - linePoint[1];
                        lineAngle = getFollowedLineAngle() - currentOrientation.firstAngle;
                    }
                }
                telemetry.addData("Projection line angle",lineAngle);
                result = AngleUtils.projectPoint(lineAngle, cm_ahead_prediction * pixels_per_cm, linePoint);
                break;
            case ANGLES_FROM_NODE:
                if (nodeCenter != null) {
                    double[] shiftedNode = AngleUtils.getPointPos(nodeCenter);
                    shiftedNode[1] = nodePipeline.getFindNodeImageOutput().height() - shiftedNode[1];
                    result = shiftedNode;
                } else
                    result = new double[2];
                break;
        }
        return result;
    }


    public String arrayToString(double[] array){
        String result = " ";
        for (Object obj : array){
            result += obj + ",";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }
    public String arrayToString(Point[] array){
        String result = " ";
        for (Object obj : array){
            if (obj == null)
                result += "null,";
            else
                result += obj + ",";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }
}
