package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.teamcode.AngleUtils;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class DisplayElement {
    public enum elementType{
        POINT,
        VECTOR,
        SHAPE
    }

    public elementType myType;
    public Point[] myPoints;
    public int mySize = 8;
    public String myText;
    public Scalar myColor;


    public DisplayElement(Point pos,Scalar color){
        myType = elementType.POINT;
        Point[] points = {pos};
        myPoints = points;
        myColor = color;
    }
    public DisplayElement(Point pos,int markerSize,Scalar color){
        myType = elementType.POINT;
        Point[] points = {pos};
        mySize = markerSize;
        myPoints = points;
        myColor = color;
    }

    public DisplayElement(Point p1, Point p2, Scalar color){
        Point[] points = {p1,p2};
        myPoints = points;
        myColor = color;
        myType = elementType.VECTOR;
    }

    public DisplayElement(Point p1, double angle, double length, Scalar color){
        myColor = color;
        Point p2 = new Point(AngleUtils.projectPoint(angle/180*Math.PI,length,AngleUtils.getPointPos(p1)));
        Point[] points = {p1,p2};
        myPoints = points;
        myType = elementType.VECTOR;
    }

    public DisplayElement(Point[] points, Scalar color){
        myType = elementType.SHAPE;
        myColor = color;
        myPoints = points;
    }

    public String toString(){
        return "Type: " + myType.name() + ", Points: " + AngleUtils.arrayToString(myPoints) + "Color: " + myColor;
    }

}
