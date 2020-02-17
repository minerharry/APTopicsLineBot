package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="line_bot")
public class LineBot extends OpMode {

    private RobotLocation encoderOnlyLocation = new RobotLocation();
    private RobotLocation gyroEncoderLocation = new RobotLocation();
    private RobotLocation targetLocation = new RobotLocation();


    enum WheelType {
        OmniWheels(4.0),
        StealthWheels(5.0),
        TetrixMaxSmall(3.7),
        TetrixMaxBig(4.9);

        double myRadius;
        WheelType(double radius) {myRadius = radius;}
        double getRadiusCm() {return myRadius;}
    }
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

    private BNO055IMU imu;


    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor[] motors;

    private double leftLastPos;
    private double rightLastPos;

    private Orientation lastOrientation = new Orientation();
    private AxesOrder axes = AxesOrder.ZYX;


    private static final double WHEEL_DISTANCE = 29.4; //in cm: should be in same units as wheel diameter
    private static final int TICKS_PER_ROTATION = 1680;
    private static final WheelType WHEEL_TYPE = WheelType.OmniWheels;
    private static final double WHEEL_RADIUS =  WHEEL_TYPE.getRadiusCm(); //in cm: should be in same units as wheel distance
    private static final double UNITS_PER_TICK = WHEEL_RADIUS/TICKS_PER_ROTATION;

    private boolean sim = false;
    private double simL;
    private double simR;
    private double simVL;
    private double simVR;
    private double[] simPos;
    private double simRot;

    private boolean drive_mode = false;
    private boolean auto = true;
    private boolean last_a = false;
    private double speed = 0.5;

    @Override
    public void init() {

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        FL = hardwareMap.get(DcMotor.class,motorNames.FrontLeft.getName());
        FR = hardwareMap.get(DcMotor.class,motorNames.FrontRight.getName());
        BL = hardwareMap.get(DcMotor.class,motorNames.BackLeft.getName());
        BR = hardwareMap.get(DcMotor.class,motorNames.BackRight.getName());
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


        leftLastPos = getLeftEncoderAvg();
        rightLastPos = getRightEncoderAvg();
        telemetry.addData("Left Encoder: ", leftLastPos);
        telemetry.addData("Right Encoder: ", rightLastPos);
        if (sim) {
            simL = 0;
            simR = 0;
            simVL = 10;
            simVR = 10;
            simPos = new double[2];
            simRot = 0;
        }
        double[] pos = {100,100};
        targetLocation.setPos(pos);
    }

    private double getLeftEncoderAvg(){
        double front = FL.getCurrentPosition();
        double back = BL.getCurrentPosition();
        return (front + back) / 2;
    }

    private double getRightEncoderAvg(){
        double front = FR.getCurrentPosition();
        double back = BR.getCurrentPosition();
        return (front + back) / 2;
    }

    @Override
    public void loop() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Current Gyro angle: ",angles.firstAngle);
        /*for (DcMotor motor : motors) {
            //motor.setPower(1.0);
        }*/
        double targetangle = getTargetAngle(this.targetLocation.getPos());
        telemetry.addData("Target Angle: ", targetangle);
        double distance = pointDistance(this.gyroEncoderLocation.getPos(), this.targetLocation.getPos());
        telemetry.addData("Distance to target: ",distance);
        if (auto){
// adjust relative speed based on heading error.

            if (distance > 10) {
                double error = getError(targetangle, angles.firstAngle);
                double steer = getSteer(error, 0.3);


                double idealSpeed = Math.min(1.0,distance/30);

                double leftSpeed = speed + steer;
                double rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > idealSpeed) {
                    leftSpeed = leftSpeed/(max)*idealSpeed;
                    rightSpeed = rightSpeed/(max)*idealSpeed;
                }
                double leftPower = leftSpeed;
                double rightPower = rightSpeed;
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
        }
        else {
            if (gamepad1.a && !last_a) {
                drive_mode = !drive_mode;
            }
            last_a = gamepad1.a;
            if (drive_mode) {
                double leftPower = gamepad1.left_stick_y;
                double rightPower = gamepad1.right_stick_y;

                FL.setPower(leftPower);
                BL.setPower(leftPower);
                FR.setPower(rightPower);
                BR.setPower(rightPower);
            } else {
                double vPower = gamepad1.left_stick_y;
                double hPower = gamepad1.right_stick_x;

                FL.setPower(vPower / 2 - hPower / 2);
                BL.setPower(vPower / 2 - hPower / 2);
                FR.setPower(vPower / 2 + hPower / 2);
                BR.setPower(vPower / 2 + hPower / 2);
            }
        }




        double deltaTheta = angles.firstAngle-lastOrientation.firstAngle;
        double currentLeftPos = getLeftEncoderAvg();
        double currentRightPos = getRightEncoderAvg();
        double[][] deltas = getOffsets(leftLastPos,currentLeftPos,rightLastPos,currentRightPos);
        encoderOnlyLocation.incrementOffsets(deltas[0],deltas[1][0]);
        deltas = getOffsets(leftLastPos,currentLeftPos,rightLastPos,currentRightPos,deltaTheta);
        gyroEncoderLocation.incrementOffsets(deltas[0],deltas[1][0]);
        lastOrientation = angles;

        leftLastPos = currentLeftPos;
        rightLastPos = currentRightPos;
        telemetry.addData("Left Encoder: ", leftLastPos);
        telemetry.addData("Right Encoder: ", rightLastPos);

        telemetry.addData("Encoder-based Location: ", encoderOnlyLocation.getInfo());
        telemetry.addData("gyro-based Location: ", gyroEncoderLocation.getInfo());



        if (sim) {

            double[][] simDeltas = getOffsets(simL, simL+simVL, simR, simR + simVR);
            double[] simdPos = simDeltas[0];
            double simdTheta = simDeltas[1][0];
            double[] simrotatedDPos = {simdPos[0]*Math.cos(simRot)-simdPos[1]*Math.sin(simRot),simdPos[0]*Math.sin(simRot)+simdPos[1]*Math.cos(simRot)};
            simPos[0] += simrotatedDPos[0]; simPos[1] += simrotatedDPos[1];
            simRot += simdTheta;
            simL += simVL;
            simR += simVR;
            telemetry.addData("simL: ", simL);
            telemetry.addData("simR: ", simR);
            telemetry.addData("simVL: ", simVL);
            telemetry.addData("simVR: ", simVR);
            telemetry.addData("simRot: ", simRot);
            telemetry.addData("simPos: ", simPos[0] + ", " + simPos[1]);
        }


    }

    public double[][] getOffsets(double pastLeft, double currentLeft, double pastRight, double currentRight) {
        double dL = (currentLeft - pastLeft)*UNITS_PER_TICK*2*Math.PI; //delta left encoder
        double dR = (currentRight - pastRight)*UNITS_PER_TICK*2*Math.PI; //delta right encoder
        if (dL == dR) {
            double[][] out = {{0,dL},{0}};
            return out;
        }

        telemetry.addData("dL: ",dL);
        telemetry.addData("dR: ",dR);

        int outside = (Math.abs(dR) > Math.abs(dL) ? -1 : 1);

        //central radius - radius of the circle halfway between wheels
        double rC = WHEEL_DISTANCE/2*(dR + dL) / ((dL - dR))*outside;
        telemetry.addData("encoder RC: ", rC);

        //known arclength, R, over known radius of R, which is shifted from center by a certain amount
        double theta = 0;
        if (dL == 0) {
            theta = dR/(rC - outside*WHEEL_DISTANCE/2);
        }
        else {
            theta = dL/(rC + outside*WHEEL_DISTANCE/2);
        }

        double[] dP = {rC * (1-Math.cos(theta)),rC*Math.sin(theta)};

        double[][] out = {dP,{-theta*outside}};
        return out;
    }

    public double[][] getOffsets(double pastLeft, double currentLeft, double pastRight, double currentRight,double gTheta) {
        double dL = (currentLeft - pastLeft)*UNITS_PER_TICK*2*Math.PI; //delta left encoder
        double dR = (currentRight - pastRight)*UNITS_PER_TICK*2*Math.PI; //delta right encoder
        if (dL == dR || gTheta == 0) {
            double[][] out = {{0,(dL+dR)/2.0},{0}};
            return out;
        }

        telemetry.addData("dL: ",dL);
        telemetry.addData("dR: ",dR);

        int outside = (Math.abs(dR) > Math.abs(dL) ? -1 : 1);

        double theta = gTheta / outside;

        double outsideArc = (outside > 0 ? dL : dR);

        //central radius - radius of the circle halfway between wheels
        double rC = -outside*WHEEL_DISTANCE/2 + outsideArc/theta;
        telemetry.addData("gyro RC: ", rC);

        double[] dP = {rC * (1-Math.cos(theta)),rC*Math.sin(theta)};

        double[][] out = {dP,{gTheta}};
        return out;
    }

    private double getError(double targetAngle,double currentAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - currentAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double pointDistance(double[] p1, double[] p2){
        return Math.sqrt(Math.pow(p1[0]-p2[0],2)+Math.pow(p1[1]-p2[1],2));
    }

    public double getTargetAngle(double[] targetLocation) {
        double[] myLocation = this.gyroEncoderLocation.getPos();
        return Math.atan2(targetLocation[1]-myLocation[1],targetLocation[0]-myLocation[0]) - Math.PI/2;
    }







}
