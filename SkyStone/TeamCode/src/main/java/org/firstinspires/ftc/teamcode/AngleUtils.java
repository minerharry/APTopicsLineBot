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

    public static double[] getPointPos(Point p) {
        System.out.println("Point: " + p);
        double[] result = {p.x,p.y};
        return result;
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

    public static double distance(double[] p1, double[] p2) {
        return Math.sqrt(Math.pow(p1[0]-p2[0],2)+Math.pow(p1[1]-p2[1],2));
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


}
