package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class AngleUtils {

    //Some parts of this class very important, because angle system is that 0 is forward and right is positive,
    //different from matheamtically. All functions take in and return such angles unless specified otherwise

    //if the angles are exactly opposite each other, will default to the smaller-numbered midAngle
    public static double midAngle(double angle1, double angle2) {
        angle1 = (angle1 + Math.PI*4)%(Math.PI*2);
        angle2 = (angle2 + Math.PI*4)%(Math.PI*2);
        double distance = shortestAngleBetween(angle1,angle2);
        return ((angle1 + distance/2) + Math.PI*4)%(Math.PI*2);
    }

    public static double weightedAvgAngle(double angle1, double weight1, double angle2, double weight2){
        angle1 = (angle1 + Math.PI*4)%(Math.PI*2);
        angle2 = (angle2 + Math.PI*4)%(Math.PI*2);
        double distance = shortestAngleBetween(angle1,angle2);
        return ((angle1 + weight1*distance/(weight1+weight2)) + Math.PI*4)%(Math.PI*2);
    }

    public static double[] getPointPos(Point p) {
        //System.out.println("Point: " + p);
        double[] result = {p.x,p.y};
        return result;
    }

    public static double wrapSign(double inAngle, double wrapThreshold){
        inAngle %= Math.PI*2;
        if (inAngle > wrapThreshold) {
            inAngle -= Math.PI * 2;
        }
        return inAngle;
    }

    public static double wrapSign(double inAngle){
        return wrapSign(inAngle,Math.PI);
    }

    public static double pythagorean(double x, double y){
        return Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
    }

    //returns the index of the nearest angle in a given list to some target angle, using shortest angle between
    public static int nearestAngle(double target, double[] angles){
        int shortestIndex = -1;
        double minDistance = Math.PI*2;
        for (int i = 0; i < angles.length; i++)
        {
            double distance = Math.abs(shortestAngleBetween(target,angles[i]));
            if (distance < minDistance)
            {
                minDistance = distance;
                shortestIndex = i;
            }
        }
        return shortestIndex;
    }

    public static String arrayToString(double[] array){
        String result = " ";
        for (Object obj : array){
            result += obj + ",";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }

    public static String arrayToString(double[][] array){
        String result = " ";
        for (double[] obj : array){
            result += arrayToString(obj) + ",";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }

    public static String arrayToString(Point[] array){
        String result = " ";
        for (Point obj : array){
            result += "[" + obj.x + ", " + obj.y + "],";
        }
        return "["+result.substring(0,result.length()-1) + " ]";
    }

    public static double distance(double[] p1, double[] p2) {
        return pythagorean(p2[0]-p1[0],p2[1]-p1[1]);
    }

    //returns multiplied by shortestDirectionBetween, effectively
    public static double shortestAngleBetween(double angle1, double angle2) {
        angle1 = (angle1 + Math.PI*4)%(Math.PI*2); //normalize angles
        angle2 = (angle2 + Math.PI*4)%(Math.PI*2); //normalize angles
        double angleDiff = ((angle1-angle2)+Math.PI*4)%(Math.PI*2); //get clockwise distance and normalize
        if (angleDiff == 0) {
            return 0;
        }
        if (angleDiff > Math.PI) {
            return Math.PI*2-angleDiff;
        }
        return -angleDiff;
    }


    //lineAngle in global angles
    public static double perpendicularDistance(double[] lineOrigin, double lineAngle, double[] point){
        double interiorAngle = lineAngle-angleBetweenPoints(lineOrigin,point);
        return Math.sin(interiorAngle)*distance(lineOrigin,point);
    }
    //lineAngle in global angles
    public static double parallelDistance(double[] lineOrigin, double lineAngle, double[] point){
        double interiorAngle = lineAngle-angleBetweenPoints(lineOrigin,point);
        return Math.cos(interiorAngle)*distance(lineOrigin,point);
    }

    public static int shortestDirectionBetween(double angle1, double angle2) {
        angle1 = (angle1 + Math.PI*4)%(Math.PI*2);
        angle2 = (angle2 + Math.PI*4)%(Math.PI*2);
        double angleDiff = ((angle1-angle2)+Math.PI*4)%(Math.PI*2);
        if (angleDiff == 0) {
            return 0;
        }
        if (angleDiff > Math.PI) {
            return 1;
        }
        return -1;
    }


    //returns global angle from p1 to p2, measured in global axis system
    public static double angleBetweenPoints(double[] p1, double[] p2)
    {
        double dx = p2[0]-p1[0];
        double dy = p2[1]-p1[1];
        double regAngle = Math.atan2(dy,dx);
        return (regAngle + Math.PI*4) % (Math.PI*2);
    }

    public static double[] projectPoint(double angle, double distance, double[] startPoint) {
        double dx = distance * Math.cos(angle);
        double dy = distance * Math.sin(angle);
        double[] result = {dx + startPoint[0], dy + startPoint[1]};
        return result;
    }
    public static double[] projectPoint(double angle, double distance) {
        double[] startPoint = {0,0};
        return projectPoint(angle,distance,startPoint);
    }

    public static double angleBetweenPoints(Point p1, Point p2) {
        return angleBetweenPoints(getPointPos(p1),getPointPos(p2));
    }

    public static double angleBetweenPoints(double[] p1, Point p2) {
        return angleBetweenPoints(p1,getPointPos(p2));
    }
    public static double angleBetweenPoints(Point p1, double[] p2) {
        return angleBetweenPoints(getPointPos(p1),p2);
    }

    public static double[] pointBetween(double[] p1, double[] p2){
        double[] result = {(p1[0] + p2[0])/2,(p1[1]+p2[1])/2};
        return result;
    }

    //rotates ccw by the input angle
    public static double[] rotatePoint(double[] p1, double angle){
        double x = p1[0]*Math.cos(angle)-p1[1]*Math.sin(angle);
        double y = p1[0]*Math.sin(angle)+p1[1]*Math.cos(angle);
        double[] result = {x,y};
        return result;
    }


}
