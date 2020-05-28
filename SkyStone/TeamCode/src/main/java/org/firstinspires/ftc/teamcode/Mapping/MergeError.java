package org.firstinspires.ftc.teamcode.Mapping;

import java.io.Serializable;

public class MergeError implements Serializable {

    private static final long serialVersionUID = 13L; //doesn't work without it; for some reason won't serialize correctly without forcing it to recognize same versions
    public enum ErrorType {
        BRANCH_ERROR,
        NODE_ERROR
    }
    public enum BranchError {
        MISMATCHED_ENDS,
        MISALIGNED_STUBS
    }
    public enum NodeError {
        TOO_FAR,
        BAD_ANGLES,
    }

    private ErrorType errorType;
    public ErrorType getErrorType(){
        return errorType;
    }

    public Branch getBranchSource() {
        return branchSource;
    }

    public Branch getOtherBranch() {
        return otherBranch;
    }

    public BranchError getBranchError() {
        return branchError;
    }

    private Branch branchSource;
    private Branch otherBranch;
    private BranchError branchError;


    public Node getNodeSource() {
        return nodeSource;
    }

    public Node getOtherNode() {
        return otherNode;
    }

    public NodeError getNodeError() {
        return nodeError;
    }

    private Node nodeSource;
    private Node otherNode;
    private NodeError nodeError;

    private boolean locatable; //if locatable, means the error is resolvable by going to a location
                                //otherwise, means nonresolvable
    private boolean resolved = false;

    public boolean hasLocation(){
        return locatable;
    }
    public boolean isResolved(){
        return resolved;
    }




    public MergeError(Branch branch, Branch other, Node end1, Node end2) {
        branchSource = branch;
        otherBranch = other;
        nodeSource = end1;
        otherNode = end2;
        branchError = BranchError.MISMATCHED_ENDS;
        errorType = ErrorType.BRANCH_ERROR;
        locatable = true;
    }

    public MergeError(Branch branch, Branch other, boolean alignDirection, BranchError error) {
        branchSource = branch;
        otherBranch = other;
        branchError = error;
        errorType = ErrorType.BRANCH_ERROR;
        locatable = false;
        switch (branchError){
            case MISMATCHED_ENDS:
                //TODO: implement intelligent branch mismatch detection
                System.out.println("ERROR: Used intelligent branch mismatch detection before implementation");
                locatable = true;
                break;
        }
    }

    public MergeError(Node source, Node other, NodeError error){
        nodeSource = source;
        otherNode = other;
        nodeError = error;
        errorType = ErrorType.NODE_ERROR;
        locatable = false;
    }

    public String toString(){
        switch (errorType){
            case NODE_ERROR:
                return "Node Error: " + nodeError.name() + " with nodes " + nodeSource + " and " + otherNode;
            case BRANCH_ERROR:
                return "Branch Error: " + branchError.name() + " between branches " + branchSource + " and " + otherBranch + (branchError == BranchError.MISMATCHED_ENDS ? ", with ends " + nodeSource + " and " + otherNode + " mismatched" : "");
        }
        return "";
    }


}
