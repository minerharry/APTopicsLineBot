package org.firstinspires.ftc.teamcode.Replay;

import java.io.Serializable;

import org.firstinspires.ftc.teamcode.RobotLocation;
import org.firstinspires.ftc.teamcode.component_tests.NavigateMappingTester;

public class ReplayTickInformation extends ReplayInformation implements Serializable {

	public RobotLocation location;
	public double predictedAccError;
	public boolean followingLine;

	public double[][] lineCenters; //pixel units
	public double[] lineAngles; //global angles, relative to world orientation NOTE: nondirectional when not at node
	public double[] lineLengths; //total length of the line

	public int followedLineIndex;
	public boolean validLine; //whether the robot sees a valid line close enough to the targetLineAngle

	public double targetLineAngle; //global angle, relative to world orientation 

	public double[] nodeCenter; //pixel units


	public ReplayTickInformation(double time, RobotLocation loc, double error, boolean following) {
		super(time, ReplayTickInformation.class);
		location = loc;
		predictedAccError = error;
		followingLine = following;
	}

	public ReplayTickInformation(double time, RobotLocation loc, double error, boolean following, double[][] centers,
								 double[] angles, double[] lengths, double targetAngle,
								 int index, boolean valid, double[] node){
		super(time, ReplayTickInformation.class);
		location = loc;
		predictedAccError = error;
		followingLine = following;
		lineCenters = centers;
		lineAngles = angles;
		lineLengths = lengths;
		targetLineAngle = targetAngle;
		followedLineIndex = index;
		validLine = valid;
		nodeCenter = node;
	}


	protected ReplayTickInformation(double time) {
		super(time);
		location = new RobotLocation();
		predictedAccError = 0;
	}

	// generates a blank, null-state mappingInformation that is usable but empty
	public static ReplayTickInformation blank(double time) {
		return new ReplayTickInformation(time);
	}

}
