package org.firstinspires.ftc.teamcode.Mapping;

import static org.firstinspires.ftc.teamcode.AngleUtils.midAngle;

class Branch {
    private Node myStart;
    private Node myEnd; //arbitrary which is 'start' and end; branch has no required direction, but it specifies the direction of the angle
    private double myAngle; //angle from start to end, 0 being forward and right being positive
    private double myApproxLength; //unused atm, but approximately calculable
    private boolean isStub = false;

    public Branch(Node start, Node end) {
        myStart = start;
        myEnd = end;
        myAngle = angleBetween(myStart.getApproxPos(),myEnd.getApproxPos());
        myApproxLength = distance(myStart.getApproxPos(),myEnd.getApproxPos());
    }

    public Branch(Node start, Node end, double angle) {
        myStart = start;
        myEnd = end;
        myAngle = angle;
        myApproxLength = distance(myStart.getApproxPos(), myEnd.getApproxPos());
    }

    public Branch(Node start, double angle) {
        myStart = start;
        myAngle = angle;
        isStub = true;
    }

    //returns whether the merge was successful; dependent on whether the other branch is between the same nodes
    public boolean mergeFrom(Branch other) {

        if (other.getIsStub() || (other.getEnd().isEqual(myStart) && other.getStart().isEqual(myEnd)) || (other.getStart().isEqual(myStart) && other.getEnd().isEqual(myEnd))) {
            if (other.getStart().isEqual(myStart)) {
                mergeAngle(other.getAngle());
            } else if (other.getStart().isEqual(myEnd)) {
                mergeAngle(other.getAngle() + Math.PI);
            } else {
                return false;
            }
        }
        return true;
    }

    public static Branch mergeStubs(Branch stub1, Branch stub2) {
        assert stub1.getIsStub();
        assert stub2.getIsStub();
        double a1 = stub1.getAngle();
        double a2 = stub2.getAngle() + Math.PI;
        double avgAngle = midAngle(a1,a2);
        return new Branch(stub1.getStart(),stub2.getStart(), avgAngle);
    }



    public void mergeAngle(double otherAngle) {

    }

    public static double angleBetween(double[] from, double[] to) {
        double tempAngle = Math.atan2(to[1]-from[1],to[0]-from[0]);
        tempAngle = Math.PI/2 - tempAngle;
        return (tempAngle + Math.PI*4) % (Math.PI*2);
    }

    public static double distance(double[] p1, double[] p2) {
        return Math.sqrt(Math.pow(p1[0]-p2[0],2) + Math.pow(p1[1]-p2[1],2));
    }

    public Node getStart() {
        return myStart;
    }

    public Node getEnd(){
        return myEnd;
    }

    public double getAngle() {
        return myAngle;
    }

    public boolean getIsStub() {
        return isStub;
    }

}
