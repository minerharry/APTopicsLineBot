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
import org.firstinspires.ftc.teamcode.AngleSystem;
import org.firstinspires.ftc.teamcode.AngleUtils;
import org.firstinspires.ftc.teamcode.Mapping.Branch;
import org.firstinspires.ftc.teamcode.Mapping.NBMap;
import org.firstinspires.ftc.teamcode.Mapping.Node;
import org.firstinspires.ftc.teamcode.Pipelines.BetterOpenCVPipeline;
import org.firstinspires.ftc.teamcode.Pipelines.BlueLineFinder;
import org.firstinspires.ftc.teamcode.Pipelines.DisplayElement;
import org.firstinspires.ftc.teamcode.Pipelines.GreenNodeFinder;
import org.firstinspires.ftc.teamcode.Pipelines.JointPipeline;
import org.firstinspires.ftc.teamcode.Replay.ReplayMappingInformation;
import org.firstinspires.ftc.teamcode.Replay.ReplayTickInformation;
import org.firstinspires.ftc.teamcode.RobotLocation;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Stack;

@TeleOp(name="Navigation Mapping Test")
public class NavigateMappingTester extends OpMode {

    //Position stuff
    private static final double WHEEL_DISTANCE = 29.4; //in cm: should be in same units as wheel diameter
    private static final int TICKS_PER_ROTATION = 1680;
    private static final double WHEEL_RADIUS =  4; //in cm: should be in same units as wheel distance
    private static final double UNITS_PER_TICK = WHEEL_RADIUS/TICKS_PER_ROTATION*Math.PI*2;

    /** Position Approximation Stuff **/
    private RobotLocation myLocation = new RobotLocation();
    private RobotLocation simLocation = new RobotLocation();
    private RobotLocation gyroLeftLocation = new RobotLocation();
    private RobotLocation gyroRightLocation = new RobotLocation();

    private double leftLastPos;
    private double rightLastPos;


    private final double errorPerUnitDistance = 0.1;
    private double predictedPositionError = 0; //Should only ever increase

    private double distanceTraveled = 0;

    private int tickCounter = 0;


    /**Mapping**/
    private NBMap map;
    public Node lastNode;
    public Branch nextBranch;
    public Stack<Branch> navPath;


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

    /**nav constants**/
    private final double auto_movement_speed = 0.5;

    private final double line_speed = 0.35;
    private final double line_steer_coeff = 0.4; //how quickly it goes from min steer to max steer based on angle
    private final double line_max_steer = 2; //maximum steer that will be applied
    private final double line_turn_angle = 1*Math.PI/4;
    private final double line_reverse_angle = 4*Math.PI/5;

    private final double line_angle_error_threshold = Math.PI/10;

    private final double node_speed = 0.18;
    private final double node_steer_coeff = 0.4; //how quickly it goes from min steer to max steer based on angle
    private final double node_max_steer = 0.6; //maximum steer that will be applied
    private final double node_forward_angle = 2*Math.PI/5;
    private final double node_reverse_angle = 3*Math.PI/5;
    private final double node_distance_threshold = 12; //how close it must get to the node to be considered 'on' the node

    private final double found_node_perpendicular_threshold = 50; //in pixels
    private final double found_node_parallel_threshold = 20;

    private final double pixels_per_cm = 240/11.1;
    private final double cm_ahead_prediction = 7;

    //angle system stuff
    private final AngleSystem OpenCV_angles = new AngleSystem(0, false,true);
    private final AngleSystem robot_angles = new AngleSystem(Math.PI/2,false);



    //Gyro init stuff
    private BNO055IMU imu;
    private Orientation currentOrientation = new Orientation();
    private Orientation lastOrientation = new Orientation();
    private AxesOrder axes = AxesOrder.ZYX;

    //Opencv & pipeline stuff
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

    private boolean mapped = false;
    private boolean auto = false;
    private AutoMode currentMode = AutoMode.FOLLOW_LINE;
    private double lineApproxAngle = 0;
    private int nodeSeenCounter = 0;
    private static final int nodeSeenThreshold = 10;

    private int nodeNotSeenCounter = 0;
    private static final int nodeNotSeenThreshold = 15;
    private boolean from_node = false;

    private ArrayList<DisplayElement> displayElements = new ArrayList<>();

    private double displaySize = 220;
    private double mapSize = 150;


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
        BetterOpenCVPipeline[] pipelines = {nodePipeline,linePipeline};
        jointPipeline = new JointPipeline(pipelines,1);
        webcam.setPipeline(jointPipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        map = new NBMap();
        System.out.println("Starting Replay");

    }

    @Override
    public void loop(){
        telemetry.addData("Currently Rendered image",jointPipeline.getDisplayedStageName());

        lineAngles = linePipeline.getLineAngles();
        lineCenters = linePipeline.getLineCenters();
        nodeCenter = nodePipeline.getFindNodeCenter();

        displayElements = new ArrayList<>();


        //get gyro angle
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Current Gyro angle: ",currentOrientation.firstAngle);

        //update current position
        updatePosition();


        double[] targetPoint = getTargetPoint();
        double[] robotCenter = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2}; //would need to invert y, but exactly halfway in the image, so not needed
        double targetAngle =AngleUtils.wrapSign(robot_angles.fromGlobal(AngleUtils.angleBetweenPoints(robotCenter,targetPoint)));
        double targetDistance = AngleUtils.distance(targetPoint,robotCenter);

        //display approxLineAngle and chosen line
        double opencvAngle = (robot_angles.convertTo(lineApproxAngle + currentOrientation.firstAngle,OpenCV_angles));
        Point p1 = new Point(robotCenter);
        Point p3 = new Point();
        if (getFollowedLineIndex() >= 0 && lineCenters.length > 0){
            p3 = lineCenters[getFollowedLineIndex()];
        }

        Scalar color = new Scalar(255,0,0);
        displayElements.add(new DisplayElement(p1,color));
        displayElements.add(new DisplayElement(p1,opencvAngle,10,color));
        displayElements.add(new DisplayElement(p3,color));
        displayElements.add(new DisplayElement(new Point(targetPoint[0],linePipeline.maskOutput().height()-targetPoint[1]),color));




        /*telemetry.addData("Test distance", AngleUtils.shortestAngleBetween(1.5707,1.588));
        telemetry.addData("Line Angles: ", arrayToString(lineAngles));
        telemetry.addData("Line Centers: ", arrayToString(lineCenters));

        telemetry.addData("Target Pos: ", arrayToString(targetPoint));
        telemetry.addData("Target angle: ", targetAngle);
        telemetry.addData("Target distance: ", targetDistance);*/

        double[] shiftedAngles = getShiftedLineAngles(nodeCenter != null);
        double[] realAngles = new double[shiftedAngles.length];
        for (int i = 0; i < realAngles.length; i++)
        {
            realAngles[i] = robot_angles.fromGlobal(shiftedAngles[i]);
        }
        /*
        telemetry.addData("robot angled lines", arrayToString(realAngles));
        telemetry.addData("robot line approx Angle", (lineApproxAngle));

        telemetry.addData("global angled lines", arrayToString(shiftedAngles));
        telemetry.addData("global line approx Angle", robot_angles.toGlobal(lineApproxAngle));
        */

        double[] test1 = {2,-0.5};
        double targetTest = gamepad1.left_stick_y;
        //telemetry.addData("Closest Angle", test1[AngleUtils.nearestAngle(targetTest,test1)]);



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
                    boolean going_away = false;
                    double perpDist = 100;
                    if (lineAngles != null && lineAngles.length > 0 && getFollowedLineError()<line_angle_error_threshold ) {
                        double lineAngle = getFollowedLineAngle() - currentOrientation.firstAngle;
                        perpDist = AngleUtils.perpendicularDistance(robotCenter, lineAngle, targetPoint);
                        going_away = (perpDist < 0 == (AngleUtils.wrapSign(robot_angles.fromGlobal(lineAngle)) < 0));
                        telemetry.addData("Going away line angle", lineAngle);
                        telemetry.addData("robot going away line angle",AngleUtils.wrapSign(robot_angles.fromGlobal(lineAngle)));
                        telemetry.addData("Going away perp dist", perpDist);
                        telemetry.addData("Going away", going_away);
                    }

                    steer = Range.clip(targetAngle*line_steer_coeff,-1,1)* line_max_steer;
                    maxSpeed = 1;
                    speed = (going_away ? 35/Math.max(Math.abs(perpDist),35): 1)*(Math.abs(targetAngle) < line_turn_angle ? line_speed : (Math.abs(targetAngle) > line_reverse_angle ? -0.3*line_speed : 0.05*-line_speed));
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
            if (nodeCenter != null){
                speed *= 0.74;
                steer *= 0.8;
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


        if (mapped){
            telemetry.addData("Navigation Status","Fully mapped");
        }
        //auto logic stuff
        if (auto) {
            telemetry.addData("From Node",from_node);
            telemetry.addData("From Node Counter",nodeNotSeenCounter);
            if (currentMode == AutoMode.FOLLOW_LINE) {
                if (lineAngles != null && lineAngles.length > 0) {
                    if (nodeCenter != null) {
                        double[] nodePos = AngleUtils.getPointPos(nodeCenter);
                        nodePos[1] = nodePipeline.getFindNodeImageOutput().height() - nodePos[1];
                        int targetIndex = getFollowedLineIndex();
                        double[] linePoint = AngleUtils.getPointPos(lineCenters[targetIndex]);
                        linePoint[1] = linePipeline.maskOutput().height() - linePoint[1];
//                        if (Math.abs(getFollowedLineError(false)) > line_angle_error_threshold) {
//                            linePoint = robotCenter;
//                            telemetry.addData("error","prone");
//                        }
                        double along_angle = robot_angles.toGlobal(lineApproxAngle) - currentOrientation.firstAngle;
                        double perpDist = AngleUtils.perpendicularDistance(linePoint, along_angle, nodePos);
                        double parDist = AngleUtils.parallelDistance(linePoint, along_angle, nodePos);
                        double[] along_point = AngleUtils.projectPoint(along_angle,30,linePoint);
                        displayElements.add(new DisplayElement(new Point(along_point[0],nodePipeline.getFindNodeImageOutput().height()-along_point[1]),color));
                        /*telemetry.addData("distance inputs",arrayToString(linePoint) + ", " + along_angle+ ", " + arrayToString(nodePos));
                        telemetry.addData("ParDist", parDist);
                        telemetry.addData("PerpDist", perpDist);*/

                        if (from_node){
                            if (Math.abs(perpDist) < found_node_perpendicular_threshold && parDist < found_node_parallel_threshold){
                                nodeNotSeenCounter = Range.clip(nodeNotSeenCounter - 3, 0, 100);
                            }
                            else {
                                nodeNotSeenCounter += 1;
                                if (nodeNotSeenCounter > nodeNotSeenThreshold){
                                    from_node = false;
                                }
                            }
                        }
                        else {
                            if (parDist > found_node_parallel_threshold && Math.abs(perpDist) < found_node_perpendicular_threshold) {
                                nodeSeenCounter++;
                            } else {
                                nodeSeenCounter = Range.clip(nodeSeenCounter - 2, 0, 100);
                            }
                            if (nodeSeenCounter > nodeSeenThreshold) {
                                nodeSeenCounter = 0;
                                currentMode = AutoMode.ANGLES_FROM_NODE;
                            }
                        }
                    } else {
                        if (!from_node){
                            nodeSeenCounter = Range.clip(nodeSeenCounter - 2, 0, 100);
                        }else {
                            nodeNotSeenCounter += 1;
                        }
                        if (nodeNotSeenCounter > nodeNotSeenThreshold){
                            from_node = false;
                        }
                    }
                    int index = getFollowedLineIndex();
                    if (Math.abs(getFollowedLineError()) < line_angle_error_threshold && AngleUtils.distance(AngleUtils.getPointPos(lineCenters[index]),robotCenter)<(linePipeline.maskOutput().height()/2)) {
                        lineApproxAngle = robot_angles.fromGlobal(getFollowedLineAngle());
                        telemetry.addData("Updated target angle",true);
                    }
                    else
                        telemetry.addData("Updated target angle",false);
                }
            } else if (currentMode == AutoMode.ANGLES_FROM_NODE) {
                if (nodeCenter != null)
                {
                    /*double[] nodePos = AngleUtils.getPointPos(nodeCenter);
                    nodePos[1] = nodePipeline.getFindNodeImageOutput().height() - nodePos[1];
                    int targetIndex = getFollowedLineIndex();
                    double[] linePoint = AngleUtils.getPointPos(lineCenters[targetIndex]);
                    linePoint[1] = linePipeline.maskOutput().height() - linePoint[1];
                    double along_angle = robot_angles.toGlobal(lineApproxAngle) + (gamepad1.b ? (gamepad1.y ? -currentOrientation.firstAngle : currentOrientation.firstAngle) : 0);
                    double perpDist = AngleUtils.perpendicularDistance(linePoint, along_angle, nodePos);
                    double parDist = AngleUtils.parallelDistance(linePoint, along_angle, nodePos);
                    double[] along_point = AngleUtils.projectPoint(along_angle,30,linePoint);
                    displayPoints.add(new Point(along_point[0],nodePipeline.getFindNodeImageOutput().height()-along_point[1]));
                    telemetry.addData("distance inputs",arrayToString(linePoint) + ", " + along_angle+ ", " + arrayToString(nodePos));
                    telemetry.addData("ParDist", parDist);
                    telemetry.addData("PerpDist", perpDist);*/
                }
                if (targetDistance < node_distance_threshold) {
                    if (lineAngles != null && lineAngles.length > 0) {
                        from_node = true;
                        nodeNotSeenCounter = 0;
                        double[] angles = getShiftedLineAngles(true);
                        String navStatus = "";
                        Branch prevBranch = nextBranch;
                        Node tempNode;
                        if (nextBranch == null || nextBranch.getIsStub()){
                            if (nextBranch != null){
                                navStatus = "Successfully followed stub. ";
                            }
                            double endAngle = (robot_angles.toGlobal(lineApproxAngle))%(2*Math.PI);
                            System.out.println("End angle: " + endAngle);
                            int currentAngleIndex = AngleUtils.nearestAngle(endAngle + Math.PI,angles);
                            double[] otherAngles = new double[angles.length-1];
                            int j = 0;
                            for (int i = 0; i < angles.length; i++){
                                if (i != currentAngleIndex)
                                {
                                    otherAngles[j] = angles[i];
                                    j++;
                                }

                            }
                            tempNode = (nextBranch != null ?
                                    map.createNode(nextBranch,(endAngle)%(Math.PI*2),myLocation.getPos(),otherAngles,predictedPositionError) :
                                    new Node(myLocation.getPos(),angles,predictedPositionError));
                            lastNode = map.insertNode(tempNode);
                            navStatus += "Calculating path to new stub...";
                            if (map.getStubs().size() != 0)
                                navPath = map.pathToNearestStub(lastNode);
                            else
                                navPath = map.getRoute(lastNode,map.getRandomExcludedNode(lastNode));

                            if (navPath == null){
                                telemetry.addData("Explored","All stubs");
                                System.out.println("***** Explored all stubs *****");
                                System.out.println("Map: " + map);
                                System.out.println("Current Node: " + lastNode);
                                System.out.println("Next Branch: " + nextBranch);
                                mapped = true;
                                auto = false;
                            }

                        }
                        else{
                            Node ideal_next = nextBranch.getEnd();
                            if (lastNode.equals(nextBranch.getEnd()))
                                ideal_next = nextBranch.getStart();
                            tempNode = new Node(myLocation.getPos(),angles,predictedPositionError);
                            if (ideal_next.isMergeable(tempNode)){
                                ideal_next.merge(tempNode);
                                lastNode = ideal_next;
                                navStatus = "Followed line to correct node, continuing navigation. ";
                            }
                            else{
                                lastNode = map.locateNode(tempNode,true);
                                lastNode.merge(tempNode);
                                if (map.getStubs().size() != 0)
                                    navPath = map.pathToNearestStub(lastNode);
                                else
                                    navPath = map.getRoute(lastNode,map.getRandomExcludedNode(lastNode));
                                navStatus = "Followed line to incorrect node, recalculating... ";
                            }
                            if (navPath.size() == 0){
                                if (map.getStubs().size() != 0)
                                    navPath = map.pathToNearestStub(lastNode);
                                else
                                    navPath = map.getRoute(lastNode,map.getRandomExcludedNode(lastNode));
                                navStatus = "Followed path to correct destination. Pathing to new node...";
                            }
                        }
                        System.out.println("Navigation Status: " + navStatus);


                        double next_angle = 0;
                        if (auto) {
                            nextBranch = navPath.pop();
                            Branch associatedBranch = Node.associateBranches(lastNode,tempNode).get(nextBranch);

                            next_angle = associatedBranch.getStartAngle();
                            if (!associatedBranch.getIsStub() && associatedBranch.getEnd().equals(lastNode)) {
                                next_angle = (associatedBranch.getEndAngle() + Math.PI) % (2 * Math.PI);
                            }

                            telemetry.addData("Branch angle", next_angle);
                            telemetry.addData("available angles: ", arrayToString(angles));
                            System.out.println("Branch angle: " + next_angle);
                            System.out.println("available angles: " + arrayToString(angles));
                            System.out.println("Map: " + map);
                            System.out.println("Current Node: " + lastNode);
                            System.out.println("Next Branch: " + nextBranch);


                            targetAngle = angles[AngleUtils.nearestAngle(next_angle, angles)];
                            System.out.println("Chosen target angle: " + targetAngle);

                            lineApproxAngle = robot_angles.fromGlobal(targetAngle);
                            currentMode = AutoMode.FOLLOW_LINE;
                        }
                        //Log replay
                        ReplayMappingInformation mapInfo = new ReplayMappingInformation(time,map,prevBranch,nextBranch,lastNode,navPath,navStatus,next_angle);
                        System.out.println("Logging Replay Data: " + mapInfo.getSerialString() + "--End Replay Data Log");

                    }
                }
            }
        }

        for (Point p : lineCenters){
            displayElements.add(new DisplayElement((p),new Scalar(0,255,0)));
        }


        jointPipeline.updateElements(displayElements);


        /*try{
            boolean[] bools = {true,false};
            for (int i = 0; i < 2; i++) {
                telemetry.addData("UnNodedAngles" + bools[i], arrayToString(getShiftedLineAngles(bools[i])));
                telemetry.addData("UnNodedIndex" + bools[i], getFollowedLineIndex(bools[i]));
                telemetry.addData("UnNodedAngle" + bools[i], getFollowedLineAngle(bools[i]));
                telemetry.addData("Error" + bools[i], getFollowedLineError(bools[i]));
            }
        } catch (Exception e){
            telemetry.addData("Error","errored");
        }*/



        //set last gyro angle
        lastOrientation = currentOrientation;
    }


    public void updatePosition(){
        double[] lastPosition = myLocation.getPos();
        //updatePosition(myLocation);
        updatePosition(gyroLeftLocation, true);
        updatePosition(gyroRightLocation, false);
        myLocation.setRot(gyroLeftLocation.getRot());
        myLocation.setPos(AngleUtils.pointBetween(gyroLeftLocation.getPos(),gyroRightLocation.getPos()));

        double tickDistance = AngleUtils.distance(lastPosition,myLocation.getPos());
        distanceTraveled += tickDistance;
        predictedPositionError += tickDistance * errorPerUnitDistance;

        //averageGyroLocation.setRot(gyroLeftLocation.getRot());
        //averageGyroLocation.setPos(AngleUtils.pointBetween(gyroLeftLocation.getPos(),gyroRightLocation.getPos()));
        double[] newPos = getApproxDisplayPos(myLocation.getPos());
        DisplayElement el = new DisplayElement(new Point(newPos),robot_angles.convertTo(myLocation.getRot(),OpenCV_angles),5,new Scalar(255,0,0));
        displayElements.add(el);

        for(Node n : map.getNodes()){
            double[] nodePos = getApproxDisplayPos(n.getApproxPos());
            displayElements.add(new DisplayElement(new Point(nodePos),new Scalar(0,255,0)));
        }
        for (Branch b : map.getBranches()){
            double[] startPos = getApproxDisplayPos(b.getStart().getApproxPos());
            if (b.getIsStub()){
                displayElements.add(
                        new DisplayElement(new Point(startPos),OpenCV_angles.fromGlobal(b.getStartAngle()),3,new Scalar(255,0,255))
                );
            }
            else{
                double[] endPos = getApproxDisplayPos(b.getEnd().getApproxPos());
                displayElements.add(
                        new DisplayElement(new Point(startPos),new Point(endPos),new Scalar(0,0,255))
                );
            }
        }

        tickCounter++;
        tickCounter %= (currentMode == AutoMode.FOLLOW_LINE ? 5 : 2);
        if (tickCounter == 0) {
            double[] robotCenter = {linePipeline.maskOutput().width()/2,linePipeline.maskOutput().height()/2};
            double[][] centers = new double[lineAngles.length][2];
            double[] angles = new double[lineAngles.length];
            if (nodeCenter != null)
                angles = getShiftedLineAngles(true);
            for (int i = 0; i < centers.length; i++){
                Point center = lineCenters[i];
                double[] pos = {(center.x - robotCenter[0]),(robotCenter[1]-center.y)};
                centers[i] = pos;
                if (nodeCenter == null)
                    angles[i] = OpenCV_angles.toGlobal(lineAngles[i]) + currentOrientation.firstAngle;
            }

            double[] center = null;
            if (nodeCenter != null){
                center = new double[2];
                center[0] = (nodeCenter.x - robotCenter[0]);
                center[1] = (robotCenter[1]-nodeCenter.y);
            }
            int index = (lineAngles.length == 0 ? -1 : getFollowedLineIndex());
            ReplayTickInformation tickInfo = new ReplayTickInformation(time,myLocation,predictedPositionError,currentMode == AutoMode.FOLLOW_LINE,
                    centers,angles,linePipeline.getLineLengths(),robot_angles.toGlobal(lineApproxAngle),
                    index,(lineAngles.length == 0 ? false : getFollowedLineError() < line_angle_error_threshold),center);
            System.out.println("Logging Replay Data: " + tickInfo.getSerialString() + "--End Replay Data Log");
        }
        telemetry.addData("Position Display",el);
        leftLastPos = getLeftEncoderAvg();
        rightLastPos = getRightEncoderAvg();

    }

    public double[] getApproxDisplayPos(double[] pos){
        double[] newPos = {pos[0]*displaySize/mapSize+displaySize/2,displaySize-(pos[1]*displaySize/mapSize+displaySize/2)};
        return newPos;
    }

    public void updatePosition(RobotLocation location, boolean useLeftPreference){

        double currentLeft = getLeftEncoderAvg();
        double currentRight = getRightEncoderAvg();
        double currentForward = location.getRot();

        RobotLocation change = getChange(currentLeft-leftLastPos,currentRight-rightLastPos,
                    robot_angles.fromGlobal(currentOrientation.firstAngle)-robot_angles.fromGlobal(lastOrientation.firstAngle),
                    currentForward,useLeftPreference);

        String overrideSideString = (useLeftPreference ? "Left" : "Right");
        //telemetry.addData("Change in gyro w/ " + overrideSideString + " preference Location",change);
        location.add(change);
        //telemetry.addData("gyro w/ " + overrideSideString + " preference Location",location);


    }

    public RobotLocation getChange(double dLeft, double dRight, double angleChange, double forwardAngle,boolean useLeftPreference) {
        dLeft *= UNITS_PER_TICK;
        dRight *= UNITS_PER_TICK;
        RobotLocation result = new RobotLocation();
        double dX = 0;
        double dY;
        double dRot = angleChange;

        if (dLeft == dRight || angleChange == 0) {
            dY = dLeft;
        } else {
            double rC;
            double width = WHEEL_DISTANCE;
            boolean rightOutside = (Math.abs(dRight) > Math.abs(dLeft));
            if ((useLeftPreference && dLeft != 0) || dRight == 0) {
                double rLeft = dLeft / -angleChange;
                rC = rLeft + width / 2;
            } else {
                double rRight = dRight / -angleChange;
                rC = rRight - width / 2;
            }
            double dTheta = -angleChange;//*(rightOutside ? -1 : 1);
            dX = (Math.cos(dTheta) * rC - rC) * (rightOutside ? -1 : 1);
            dY = rC * Math.sin(dTheta);
        }
        double[] unrotatedDPos = {dX, dY};
        double[] rotatedDPos = AngleUtils.rotatePoint(unrotatedDPos, -forwardAngle);
        result.setPos(rotatedDPos);
        result.setRot(dRot);
        return result;
    }

    private double getLeftEncoderAvg(){
        double front = FL.getCurrentPosition();
        double back = BL.getCurrentPosition();
        //telemetry.addData("Left Wheel Difference",front-back);
        return (front + back) / 2;
    }

    private double getRightEncoderAvg(){
        double front = FR.getCurrentPosition();
        double back = BR.getCurrentPosition();
        //telemetry.addData("Right Wheel Difference",front-back);
        return (front + back) / 2;
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
    //returns global, gyro-shifted angles
    private double[] getShiftedLineAngles(boolean useNode){
        double[] shiftedAngles = new double[lineAngles.length];
        if (useNode) {
            double[] nodePoint = {nodeCenter.x,linePipeline.maskOutput().height() - nodeCenter.y};
            for (int i = 0; i < lineCenters.length; i++) {
                double[] centerPoint = {lineCenters[i].x, linePipeline.maskOutput().height() - lineCenters[i].y};
                double angle = OpenCV_angles.toGlobal(lineAngles[i]);
                boolean sign = AngleUtils.parallelDistance(nodePoint,angle,centerPoint) <= 0;
                //telemetry.addData("Parallel line check ", AngleUtils.parallelDistance(nodePoint,angle,centerPoint));
                shiftedAngles[i] = (Math.PI * ((sign ? 1 : 0)+ (!from_node ? 1 : 0)) + angle + currentOrientation.firstAngle + Math.PI*6) % (Math.PI*2);
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

    //returns in global gyro-shifted angles
    private double getFollowedLineAngle() {
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        double[] shiftedAngles = getShiftedLineAngles(nodeCenter != null);
        double nodeDirectionInvert = 0;
        return shiftedAngles[AngleUtils.nearestAngle(targetAngle + nodeDirectionInvert,shiftedAngles)] - nodeDirectionInvert;
    }

    //returns in global gyro-shifted angles
    private double getFollowedLineAngle(boolean nodeOverride) {
        double targetAngle = robot_angles.toGlobal(lineApproxAngle);
        double[] shiftedAngles = getShiftedLineAngles(nodeOverride);
        double nodeDirectionInvert = 0;
        return shiftedAngles[AngleUtils.nearestAngle(targetAngle + nodeDirectionInvert,shiftedAngles)] - nodeDirectionInvert;
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
                //telemetry.addData("Projection line angle",lineAngle);
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
