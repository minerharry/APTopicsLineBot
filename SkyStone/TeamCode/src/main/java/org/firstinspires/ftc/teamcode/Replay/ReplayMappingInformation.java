package org.firstinspires.ftc.teamcode.Replay;

import java.io.Serializable;
import java.util.Stack;

import org.firstinspires.ftc.teamcode.Mapping.Branch;
import org.firstinspires.ftc.teamcode.Mapping.NBMap;
import org.firstinspires.ftc.teamcode.Mapping.Node;

public class ReplayMappingInformation extends ReplayInformation implements Serializable {
    public NBMap map;
    public Branch previousBranch;
    public Branch currentBranch;
    public Node currentNode;
    public Stack<Branch> navPath;
    public String navStatus;
    public double nextAngle;

    //TODO: Add node arrival snapshot

    public ReplayMappingInformation(double time, NBMap nMap, Branch prevBranch, Branch nextBranch, Node currNode,
                                    Stack<Branch> nav, String status, double next) {
        super(time, ReplayMappingInformation.class);
        map = nMap;
        previousBranch = prevBranch;
        currentBranch = nextBranch;
        currentNode = currNode;
        navPath = nav;
        navStatus = status;
        nextAngle = next;
    }

    protected ReplayMappingInformation(double time) {
        super(time);
        map = new NBMap();
        previousBranch = null;
        currentBranch = null;
        currentNode = null;
        navPath = new Stack<>();
        navStatus = "";
    }

    // generates a blank, null-state mappingInformation that is usable but empty
    public static ReplayMappingInformation blank(double time) {
        return new ReplayMappingInformation(time);
    }

}
