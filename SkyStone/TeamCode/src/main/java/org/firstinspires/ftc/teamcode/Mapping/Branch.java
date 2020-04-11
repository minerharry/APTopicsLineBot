package org.firstinspires.ftc.teamcode.Mapping;

import org.firstinspires.ftc.teamcode.AngleUtils;

import static org.firstinspires.ftc.teamcode.AngleUtils.distance;
import static org.firstinspires.ftc.teamcode.AngleUtils.midAngle;
import static org.firstinspires.ftc.teamcode.AngleUtils.weightedAvgAngle;

public class Branch {
    private Node myStart;
    private Node myEnd; //arbitrary which is 'start' and end; branch has no required direction, but it specifies the direction of the angle
    private double myAngle; //[global] average angle from start to end -- INDEPENDENT FROM THE ANGLE AT EACH END
    private double myStartAngle; //the [global] angle of the branch from the start node
    private double myEndAngle; //the [global] angle of the branch into the end node;
    private double myApproxLength; //unused atm, but approximately calculable
    private boolean isStub = false;

    private int numStartMerges = 1;
    private int numEndMerges = 1;
    private int numAngleMerges = 1;

    private int myId;
    public int getId() {
        return myId;
    }

    private static int lastId = -1;
    public static int generateId() {
        lastId++;
        return lastId;
    }

    public Branch(Node start, Node end, double startAngle, double endAngle) {
        myStart = start;
        myEnd = end;
        myStartAngle = (startAngle)%(Math.PI*2);
        myEndAngle = (endAngle)%(Math.PI*2);
        myAngle = AngleUtils.angleBetweenPoints(myStart.getApproxPos(),myEnd.getApproxPos());
        myApproxLength = distance(myStart.getApproxPos(),myEnd.getApproxPos());
        myId = generateId();
    }

    public Branch(Node start, Node end, double startAngle, double endAngle, double angle) {
        myStart = start;
        myEnd = end;
        myStartAngle = (startAngle)%(Math.PI*2);
        myEndAngle = (endAngle)%(Math.PI*2);
        myAngle = (angle)%(Math.PI*2);
        myApproxLength = distance(myStart.getApproxPos(), myEnd.getApproxPos());
        myId = generateId();
    }

    public Branch(Node start, double angle) {
        myStart = start;
        myAngle = (angle)%(Math.PI*2);
        myStartAngle = (angle)%(Math.PI*2);
        isStub = true;
        myId = generateId();
    }



    //Only able to merge branches with same start and end nodes
    //returns whether the merge was successful; dependent on whether the other branch is between the same nodes
    public boolean mergeFrom(Branch other) {
        boolean myStub = getIsStub();
        boolean otherStub = other.getIsStub();
        boolean neitherStub = !(myStub || otherStub);
        boolean alignedOrientation = false;
        boolean valid = false;
        if (getStart().isEqual(other.getStart())) {
            alignedOrientation = true;
            if (!(!neitherStub || myEnd.isEqual(other.getEnd())))
                return false;
            valid = true;
        }
        else if (myStub && otherStub){
            return false;
        }

        if (!valid && (otherStub || myStart.isEqual(other.getEnd())) && (myStub || myEnd.isEqual(other.getStart())))
            valid = true;
        else
            return false;

        //facing the same way
        if (alignedOrientation){
            myStartAngle = weightedAvgAngle(myStartAngle,numStartMerges,other.getStartAngle(),other.numStartMerges);
            numStartMerges += other.numStartMerges + 1;

            if (neitherStub){
                myEndAngle = weightedAvgAngle(myEndAngle,numEndMerges,other.getEndAngle(),other.numEndMerges);
                numEndMerges += other.numEndMerges + 1;
            } else if (!otherStub) {
                growStub(other.getEnd(),other.getAngle());
            }
        }
        else{
            if (!otherStub){
                myStartAngle = weightedAvgAngle(myStartAngle,numStartMerges,other.getEndAngle()+Math.PI,other.numEndMerges);
                numStartMerges += other.numEndMerges + 1;
            }
            if (!myStub){
                myEndAngle = weightedAvgAngle(myEndAngle,numEndMerges,other.getStartAngle(),other.numStartMerges);
                numEndMerges += other.numStartMerges + 1;
            }
            else{
                growStub(other.getStart(),other.getStartAngle() + Math.PI);
                numEndMerges = other.numStartMerges;
            }

        }
        myAngle = AngleUtils.angleBetweenPoints(myStart.getApproxPos(),myEnd.getApproxPos());
        return true;
    }


    //converts a stub to a full branch by adding an end
    public void growStub(Node end, double endAngle){
        isStub = false;
        myEnd = end;
        myEndAngle = (endAngle)%(Math.PI*2);
        myAngle = midAngle(myStartAngle,myEndAngle);
    }

    public void growStub(Node end, double endAngle, double totalAngle){
        isStub = false;
        myEnd = end;
        myEndAngle = (endAngle)%(Math.PI*2);
        myAngle = (totalAngle)%(Math.PI*2);
    }


    //stub1 being start and stub2 being end
    //rarely used
    public static Branch mergedStubs(Branch stub1, Branch stub2) {
        assert stub1.getIsStub();
        assert stub2.getIsStub();
        double a1 = stub1.getAngle();
        double a2 = stub2.getAngle() + Math.PI;
        double avgAngle = midAngle(a1,a2);
        return new Branch(stub1.getStart(),stub2.getStart(),a1, a2, avgAngle);
    }


    //start must be one of the endpoints; otherwise does nothing and returns false
    private boolean orientSelf(Node start){
        if (myStart.isEqual(start)){
            return true;
        }
        if (myEnd.isEqual(start)){
            invertSelf();
            return true;
        }
        return false;
    }

    private void invertSelf(){
        Node tempNode = myStart;
        myStart = myEnd;
        myEnd = tempNode;
        double tempAngle = (myStartAngle + Math.PI)%(Math.PI*2);
        myStartAngle = (myEndAngle + Math.PI)%(Math.PI*2);
        myEndAngle = tempAngle;
        myAngle += Math.PI;
        myAngle %= 2*Math.PI;
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

    public double getStartAngle(){
        return myStartAngle;
    }

    public double getEndAngle(){
        return myEndAngle;
    }

    public boolean getIsStub() {
        return isStub;
    }

    public void setStart(Node in){
        myStart = in;
    }

    public void setEnd(Node in){
        myEnd = in;
    }

    public boolean isEqual(Branch other){
        return myId == other.getId();
    }

    @Override
    public String toString() {
        if (getIsStub()){
            return "Stub Branch, Start: " + myStart + " with angle " + myStartAngle;
        }
        return "Branch, Start: [" + myStart + "] with angle " + myStartAngle + ", End: [" + myEnd + "] with angle " + myEndAngle + ", Overall angle: " + myAngle;
    }
}
