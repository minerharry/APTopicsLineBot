package org.firstinspires.ftc.teamcode.Mapping;

import org.firstinspires.ftc.teamcode.AngleUtils;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Node {
    private double[] approxPos;
    private int numMerges = 1;
    private boolean hasStub;
    private ArrayList<Branch> attachedLines;


    //TODO: create merging threshold
    private static final double angleMergeabilityThresholdScore = 10;
    private static final double distanceMergeabilityThreshold = 8;

    public int getId() {
        return myId;
    }

    public boolean hasStub(){
        updateHasStub();
        return hasStub;
    }

    public void updateHasStub(){
        hasStub = false;
        for (Branch b : attachedLines){
            if (b.getIsStub()){
                hasStub = true;
                return;
            }
        }
    }

    public double[] getApproxPos() {
        if (approxPos == null) {
            System.out.println("Attempting to get null approxPos");
        }
        return approxPos;
    }

    public ArrayList<Branch> getAttachedLines() {
        return attachedLines;
    }

    private int myId;

    private static int lastId = -1;

    public Node() {
        myId = generateId();
        attachedLines = new ArrayList<>();
        updateHasStub();
    }

    public Node(double[] pos) {
        myId = generateId();
        attachedLines = new ArrayList<>();
        approxPos = pos;
        updateHasStub();
    }

    public Node(double[] pos, double[] stubAngles) {
        myId = generateId();
        attachedLines = new ArrayList<>();
        approxPos = pos;
        for (double angle : stubAngles){
            attachedLines.add(new Branch(this,angle));
        }
        updateHasStub();
    }

    public boolean merge(Node other) {
        //Note: should never return false
        //Will merge both values of the node and of the branches
        approxPos[0] = (approxPos[0] * numMerges + other.getApproxPos()[0])/(numMerges+1);
        approxPos[1] = (approxPos[1] * numMerges + other.getApproxPos()[1])/(numMerges+1);



        Map<Branch,Branch> associatedBranches = associateBranches(this,other);
        for (Branch key : associatedBranches.keySet()){
            Branch value = associatedBranches.get(key);
            System.out.println("Branch Value in merge: " + value);
            if (value.getIsStub() || value.getStart().isEqual(other)){
                value.setStart(this);
            }
            if (!value.getIsStub() && value.getEnd().isEqual(other)){
                value.setEnd(this);
            }
            key.mergeFrom(value);
        }

        numMerges++;
        updateHasStub();
        return true;
    }


    //only works when starts are aligned
    public static Map<Branch,Branch> associateBranches(Node n1, Node n2){
        double[] angles1 = n1.getOutwardAngles();
        double[] angles2 = n2.getOutwardAngles();

        int length = angles1.length;

        Arrays.sort(angles1);
        Arrays.sort(angles2);

        double leastScore = Double.MAX_VALUE;
        int leastIndex = -1;
        for (int i = 0; i < length; i++){
            double score = getAngleAssociationScore(angles1,angles2,i);
            if (score < leastScore){
                leastScore = score;
                leastIndex = i;
            }
        }

        ArrayList<Branch> branches1 = n1.getAttachedLines();
        ArrayList<Branch> branches2 = n2.getAttachedLines();

        Collections.sort(branches1,new BranchAngleSorter(n1));
        Collections.sort(branches2,new BranchAngleSorter(n2));

        Map<Branch,Branch> result = new HashMap<>();
        for (int i = 0; i < length; i++) {
            result.put(branches1.get(i),branches2.get((i+leastIndex)%(length)));
        }
        return result;
    }

    public Branch getBranchTo(Node other) {
        Node outNode;
        for (Branch b : attachedLines){
            if (!b.getIsStub()){
                if (b.getStart().equals(this))
                    outNode = b.getEnd();
                else
                    outNode = (b.getStart());
                if (outNode.equals(other))
                    return b;
            }
        }
        return null;
    }

    public ArrayList<Branch> getStubs() {
        ArrayList<Branch> stubs = new ArrayList<>();
        for (Branch b : attachedLines){
            if (b.getIsStub()){
                stubs.add(b);
            }
        }
        return stubs;
    }

    private static class BranchAngleSorter implements Comparator<Branch> {

        private Node headNode;
        public BranchAngleSorter(Node source){
            headNode = source;
        }

        @Override
        public int compare(Branch o1, Branch o2) {
            double o1Angle = o1.getStartAngle();
            if (!o1.getIsStub() && o1.getEnd().isEqual(headNode))
                o1Angle = o1.getEndAngle() + Math.PI;

            double o2Angle = o2.getStartAngle();
            if (!o2.getIsStub() && o2.getEnd().isEqual(headNode))
                o2Angle = o2.getEndAngle() + Math.PI;

            o1Angle %= Math.PI*2;
            o2Angle %= Math.PI*2;

            return (o1Angle == o2Angle ? 0 : (o1Angle < o2Angle ? -1 : 1));
        }
    }

    public double[] getOutwardStubAngles(){
        ArrayList<Branch> stubs = getStubs();
        double[] result = new double[stubs.size()];
        int i = 0;
        for (Branch line : stubs){
            result[i] = line.getStartAngle();
            i++;
        }
        return result;
    }

    public double[] getOutwardAngles(){
        double[] result = new double[attachedLines.size()];
        int i = 0;
        for (Branch line : attachedLines){
            if (line.getStart().isEqual(this))
                result[i] = (line.getStartAngle())%(Math.PI*2);
            else
                result[i] = (line.getEndAngle() + Math.PI)%(Math.PI*2);
            i++;
        }
        return result;
    }



    public void attachLine(Branch line) {
        attachedLines.add(line);
        hasStub = (hasStub || line.getIsStub());
    }

    public boolean isEqual(Node other) {
        return myId == other.getId();
    }

    public boolean isMergeable(Node other) {
        if (getOutwardAngles().length != other.getOutwardAngles().length){
            return false;
        }
        double mergeScore = getBestAngleAssociationScore(this,other);
        double distance = AngleUtils.distance(getApproxPos(),other.getApproxPos());
        System.out.println("Checking mergeability of " + other + " into " + this);
        System.out.println("Angle Merge Score: " + mergeScore + ", Distance: " + distance);

        return (mergeScore < angleMergeabilityThresholdScore &&
                distance < distanceMergeabilityThreshold);
    }

    public static double getBestAngleAssociationScore(Node n1, Node n2){
        double[] angles1 = n1.getOutwardAngles();
        double[] angles2 = n2.getOutwardAngles();

        int length = angles1.length;

        Arrays.sort(angles1);
        Arrays.sort(angles2);

        double leastScore = Double.MAX_VALUE;
        for (int i = 0; i < length; i++){
            double score = getAngleAssociationScore(angles1,angles2,i);
            if (score < leastScore){
                leastScore = score;
            }
        }
        return leastScore;
    }


    public static double getAngleAssociationScore(double[] angles1, double[] angles2){
        return getAngleAssociationScore(angles1,angles2,0);
    }

    public static double getAngleAssociationScore(double[] angles1, double[] angles2, int offset){
        double score = 0;
        int length = angles1.length;
        if (angles2.length != length)
        {
            System.out.println("Error angles. Angles1: " + AngleUtils.arrayToString(angles1) + ", Angles2: " + AngleUtils.arrayToString(angles2));
            throw new RuntimeException("Error: tried to get association score for angles w/ different lengths");
        }
        for (int i = 0; i < length; i++){
            score += Math.pow(AngleUtils.shortestAngleBetween(angles1[i], angles2[(i+offset)%length]),2);
        }
        return score;
    }

    public ArrayList<Node> getAdjacentNodes(){
        ArrayList<Node> result = new ArrayList<>();
        for (Branch b : attachedLines){
            if (!b.getIsStub()){
                if (b.getStart().equals(this))
                    result.add(b.getEnd());
                else
                    result.add(b.getStart());
            }
        }
        return result;
    }

    @Override
    public String toString() {
        return "Node (id: " + myId + ") at approximately (" + approxPos[0] + ", " + approxPos[1] + "), with " + attachedLines.size() + " adjacent branches";
    }

    public static int generateId() {
        lastId++;
        return lastId;
    }
}
