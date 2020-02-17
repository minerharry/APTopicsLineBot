package org.firstinspires.ftc.teamcode.Mapping;

import java.util.ArrayList;

public class Node {
    private double[] approxPos;
    private ArrayList<Branch> attachedLines;

    public int getId() {
        return myId;
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
    }

    public Node(double[] pos) {
        myId = generateId();
        attachedLines = new ArrayList<>();
        approxPos = pos;
    }

    public boolean merge(Node other) {
        //TODO: Error correction & merging protocols, merge each branch together with appropriate line angles
        //Note: should never return false
        return true;
    }

    public void attachLine(Branch line) {
        attachedLines.add(line);
    }

    public boolean isEqual(Node other) {
        return myId == other.getId();
    }

    public static int generateId() {
        lastId++;
        return lastId;
    }
}
