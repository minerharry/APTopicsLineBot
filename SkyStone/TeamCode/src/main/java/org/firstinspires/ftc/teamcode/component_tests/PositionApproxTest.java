package org.firstinspires.ftc.teamcode.component_tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AngleSystem;
import org.firstinspires.ftc.teamcode.AngleUtils;
import org.firstinspires.ftc.teamcode.Pipelines.DisplayElement;
import org.firstinspires.ftc.teamcode.Pipelines.BlankPipeline;
import org.firstinspires.ftc.teamcode.RobotLocation;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="position approxmation tester")
public class PositionApproxTest extends OpMode {

    private static final double WHEEL_DISTANCE = 29.4; //in cm: should be in same units as wheel diameter
    private static final int TICKS_PER_ROTATION = 1680;
    private static final double WHEEL_RADIUS =  4; //in cm: should be in same units as wheel distance
    private static final double UNITS_PER_TICK = WHEEL_RADIUS/TICKS_PER_ROTATION*Math.PI*2;

    private RobotLocation myLocation = new RobotLocation();
    private RobotLocation simLocation = new RobotLocation();
    private RobotLocation gyroLeftLocation = new RobotLocation();
    private RobotLocation gyroRightLocation = new RobotLocation();
    //private RobotLocation averageGyroLocation = new RobotLocation();
    //private boolean useGyroForLocation = false;


    private double simLeftEncoders = 0;
    private double simRightEncoders = 0;




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

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor[] motors;

    private double leftLastPos;
    private double rightLastPos;

    private Orientation currentOrientation = new Orientation();
    private Orientation lastOrientation = new Orientation();
    private AxesOrder axes = AxesOrder.ZYX;

    private final AngleSystem OpenCV_angles = new AngleSystem(0, false,true);
    private final AngleSystem robot_angles = new AngleSystem(Math.PI/2,false);

    private boolean drive_mode = false;
    private boolean auto = false;
    private boolean last_a = false;
    private double speed = 0.5;



    private OpenCvWebcam webcam;
    private BlankPipeline pipeline;
    private double displaySize = 250;
    private double mapSize = 50;


    @Override
    public void init() {
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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //init gyro
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu.initialize(parameters);

        leftLastPos = getLeftEncoderAvg();
        rightLastPos = getRightEncoderAvg();




        pipeline = new BlankPipeline(false);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);



    }

    @Override
    public void loop() {

        //initial gyro stuff
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        telemetry.addData("angle",currentOrientation.firstAngle);

        //position stuffs
        updatePosition();


        if (!last_a && gamepad1.a){
            auto = !auto;
        }
        last_a = gamepad1.a;


        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        if (gamepad1.b){
            rightPower = leftPower;
        }
        if (!auto) {
            BL.setPower(leftPower*0.2);
            FL.setPower(leftPower*0.2);
            BR.setPower(rightPower*0.2);
            FR.setPower(rightPower*0.2);
        }


        displayPositions();
        //final gyro stuff
        lastOrientation = currentOrientation;
    }

    public void displayPositions(){
        RobotLocation[] displayLocations = {myLocation,gyroLeftLocation,gyroRightLocation};
        Scalar[] displayColors = {new Scalar(0,0,0), new Scalar(255,0,0), new Scalar(0,255,0),new Scalar(0,0,255),new Scalar(0,0,0),new Scalar(255,255,0)};
        if (auto){
            RobotLocation[] tempLocations = {myLocation,gyroLeftLocation,gyroRightLocation,simLocation};
            displayLocations = tempLocations;
        }
        DisplayElement[] elements = new DisplayElement[displayLocations.length+1];
        elements[0] = new DisplayElement(new Point(displaySize/2,displaySize/2), new Scalar(0,0,0));
        int i = 1;
        for (RobotLocation location : displayLocations){
            double[] newPos = {location.getPos()[0]*displaySize/mapSize+displaySize/2,displaySize-(location.getPos()[1]*displaySize/mapSize+displaySize/2)};
            double newAngle = robot_angles.convertTo(location.getRot(),OpenCV_angles);
            telemetry.addData("New Angle: ",newAngle);
            elements[i] = new DisplayElement(new Point(newPos),newAngle,15,displayColors[i]);
            telemetry.addData("Display Element " + i, elements[i].toString());
            i++;

        }
        pipeline.updateElements(elements);
    }





    public void updatePosition(){
        //updatePosition(myLocation,useGyroForLocation);
        updatePosition(gyroLeftLocation,true,true);
        updatePosition(gyroRightLocation,true,false);
        myLocation.setRot(gyroLeftLocation.getRot());
        myLocation.setPos(AngleUtils.pointBetween(gyroLeftLocation.getPos(),gyroRightLocation.getPos()));

        //averageGyroLocation.setRot(gyroLeftLocation.getRot());
        //averageGyroLocation.setPos(AngleUtils.pointBetween(gyroLeftLocation.getPos(),gyroRightLocation.getPos()));


        leftLastPos = getLeftEncoderAvg();
        rightLastPos = getRightEncoderAvg();

    }


    public void updatePosition(RobotLocation location, boolean useGyro){

        double currentLeft = getLeftEncoderAvg();
        double currentRight = getRightEncoderAvg();

        double currentForward = location.getRot();

        RobotLocation change;
        if (useGyro){
            change = getChange(currentLeft-leftLastPos,currentRight-rightLastPos,
                    robot_angles.fromGlobal(currentOrientation.firstAngle)-robot_angles.fromGlobal(lastOrientation.firstAngle),
                    currentForward);
        }
        else{
            change = getChange(currentLeft-leftLastPos,currentRight-rightLastPos,currentForward);
        }
        telemetry.addData("Change in Location",change);
        location.add(change);
        telemetry.addData("My Location",location);


    }

    public void updatePosition(RobotLocation location, boolean useGyro, boolean useLeftPreference){

        double currentLeft = getLeftEncoderAvg();
        double currentRight = getRightEncoderAvg();

        double currentForward = location.getRot();

        RobotLocation change;
        if (useGyro){
            change = getChange(currentLeft-leftLastPos,currentRight-rightLastPos,
                    robot_angles.fromGlobal(currentOrientation.firstAngle)-robot_angles.fromGlobal(lastOrientation.firstAngle),
                    currentForward,useLeftPreference);
        }
        else{
            change = getChange(currentLeft-leftLastPos,currentRight-rightLastPos,currentForward);
        }
        String overrideSideString = (useLeftPreference ? "Left" : "Right");
        telemetry.addData("Change in gyro w/ " + overrideSideString + " preference Location",change);
        location.add(change);
        telemetry.addData("gyro w/ " + overrideSideString + " preference Location",location);


    }

    public void updatePosition(RobotLocation location, boolean useGyro, double simDLeft,double simDRight){

        double currentLeft = simLeftEncoders+simDLeft*3;
        double currentRight = simRightEncoders+simDRight*3;

        double currentForward = location.getRot();

        RobotLocation change;
        if (useGyro){
            change = getChange(currentLeft-simLeftEncoders,currentRight-simRightEncoders,
                    robot_angles.fromGlobal(currentOrientation.firstAngle)-robot_angles.fromGlobal(lastOrientation.firstAngle),
                    currentForward);
        }
        else{
            change = getChange(currentLeft-simLeftEncoders,currentRight-simRightEncoders,currentForward);
        }
        telemetry.addData("Change in Sim Location",change);
        location.add(change);
        telemetry.addData("My Sim Location",location);


    }


    public RobotLocation getChange(double dLeft, double dRight, double forwardAngle){
        dLeft *= UNITS_PER_TICK;
        dRight *= UNITS_PER_TICK;
        RobotLocation result = new RobotLocation();
        double dX = 0;
        double dY = 0;
        double dRot = 0;
        if (dLeft == dRight){
            dY = dLeft;
        }
        else{
            boolean rightOutside = (Math.abs(dRight) > Math.abs(dLeft));
            double dOutside = (rightOutside ? dRight : dLeft);
            double dInside = (rightOutside ? dLeft : dRight);
            double width = WHEEL_DISTANCE;
            double rOutside = -width*dOutside/(dInside-dOutside);
            double rC = rOutside - width/2;
            double dTheta = dOutside/rOutside;
            dX = (Math.cos(dTheta)*rC-rC) * (rightOutside ? -1 : 1);
            dY = rC*Math.sin(dTheta);
            dRot = dTheta * (rightOutside ? -1 : 1);
        }

        double[] unrotatedDPos = {dX,dY};
        double[] rotatedDPos = AngleUtils.rotatePoint(unrotatedDPos,-forwardAngle);
        result.setPos(rotatedDPos);
        result.setRot(dRot);
        return result;



        /*if (dLeft == dRight){

        }
        double rightOutside = (Math.abs(dRight) > Math.abs(dLeft) ? 1 : -1);

        double rC = WHEEL_DISTANCE/2*(dRight + dLeft) / ((dRight - dLeft))*rightOutside;

        double theta;
        if (dLeft == 0) {
            theta = dRight/(rC + rightOutside*WHEEL_DISTANCE/2);
        }
        else{
            theta = dLeft/(rC - rightOutside*WHEEL_DISTANCE/2);
        }

        telemetry.addData("Theta",theta);
        double dXRot = rightOutside*rC*(1-Math.cos(rightOutside*theta));//x distance in the rotated axes
        double dYRot = rightOutside*rC*(Math.sin(rightOutside*theta));
        telemetry.addData("dXRot",dXRot);
        telemetry.addData("dYRot",dYRot);

        double dTheta = (Math.atan2(dYRot,dXRot));
        telemetry.addData("dTheta",dTheta);
        double dDistance = AngleUtils.pythagorean(dYRot,dXRot);

        double[] dPos = AngleUtils.projectPoint(dTheta+robot_angles.toGlobal(forwardAngle),dDistance);

        return new RobotLocation(dPos,dTheta); //change in position and change in angle*/


    }


    public RobotLocation getChange(double dLeft, double dRight, double angleChange, double forwardAngle) {
        telemetry.addData("This should","Not appear");
        return null;
    }

    //angleChange in robotAngles
    public RobotLocation getChange(double dLeft, double dRight, double angleChange, double forwardAngle,boolean useLeftPreference){
        dLeft *= UNITS_PER_TICK;
        dRight *= UNITS_PER_TICK;
        RobotLocation result = new RobotLocation();
        double dX = 0;
        double dY;
        double dRot = angleChange;

        if (dLeft == dRight || angleChange == 0){
            dY = dLeft;
        }
        else{
            double rC;
            double width = WHEEL_DISTANCE;
            boolean rightOutside = (Math.abs(dRight) > Math.abs(dLeft));
            if ((useLeftPreference && dLeft != 0) || dRight == 0){
                double rLeft = dLeft/-angleChange;
                rC = rLeft + width/2;
            }
            else {
                double rRight = dRight/-angleChange;
                rC = rRight  - width/2;
            }
            double dTheta = -angleChange;//*(rightOutside ? -1 : 1);
            dX = (Math.cos(dTheta)*rC-rC) * (rightOutside ? -1 : 1);
            dY = rC*Math.sin(dTheta);
        }
        double[] unrotatedDPos = {dX,dY};
        double[] rotatedDPos = AngleUtils.rotatePoint(unrotatedDPos,-forwardAngle);
        result.setPos(rotatedDPos);
        result.setRot(dRot);
        return result;



        /*if (dLeft == dRight || dTheta == 0){
            return result;
        }
        double rightOutside = (Math.abs(dRight) > Math.abs(dLeft) ? 1 : -1);

        double theta = dTheta*rightOutside;

        double outsideArc = (rightOutside > 0 ? dRight : dLeft);

        double rC = rightOutside*WHEEL_DISTANCE/2 + outsideArc/theta;;

        double dXRot = rightOutside*rC*(1-Math.cos(rightOutside*theta));//rotated x distance
        double dYRot = rightOutside*rC*(Math.sin(rightOutside*theta));
        dTheta = Math.atan2(dYRot,dXRot);
        double dDistance = AngleUtils.pythagorean(dYRot,dXRot);

        double[] dPos = AngleUtils.projectPoint(dTheta+robot_angles.toGlobal(forwardAngle),dDistance);

        return new RobotLocation(dPos,dTheta); //change in position and change in angle*/

    }

    private double getLeftEncoderAvg(){
        double front = FL.getCurrentPosition();
        double back = BL.getCurrentPosition();
        telemetry.addData("Left Wheel Difference",front-back);
        return (front + back) / 2;
    }

    private double getRightEncoderAvg(){
        double front = FR.getCurrentPosition();
        double back = BR.getCurrentPosition();
        telemetry.addData("Right Wheel Difference",front-back);
        return (front + back) / 2;
    }


}
