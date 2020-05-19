package org.firstinspires.ftc.teamcode.Replay;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Arrays;
import android.util.Base64;

/**
 * only not-abstract so that it can be initialized for finding InsertionPoint
 **/
public class ReplayInformation implements Serializable {
	public int classId;
	public double timeStamp;

	public static final Class[] subclasses = { ReplayMappingInformation.class, ReplayTickInformation.class };

	// creates a dummy ReplayInformationt that should never be used but for
	// insertion points
	protected ReplayInformation(double time) {
		this.timeStamp = time;
		this.classId = -1;
	}

	// generates a blank replay information using hidden constructor
	public static ReplayInformation blank(double time) {
		return new ReplayInformation(time);
	}

	public ReplayInformation(double time, Class type) {
		timeStamp = time;
		classId = Arrays.asList(subclasses).indexOf(type);
	}

	public String getSerialString() {
		try {
			ByteArrayOutputStream str = new ByteArrayOutputStream();
			ObjectOutputStream out = new ObjectOutputStream(str);
			out.writeObject(this);
			out.close();
			String result = Base64.encodeToString(str.toByteArray(),Base64.DEFAULT);
			result = result.replaceAll("\\n","!");
			str.close();
			return result;
		} catch (Exception e) {
			System.out.println("Error in serialization");
			e.printStackTrace();
		}
		return "";
	}

	public Class getReplayCast() {
		try {
			return subclasses[classId];
		} catch (ArrayIndexOutOfBoundsException e) {
			if (classId == -1) {
				System.out.println("Error: Attempted to get class cast on dummy ReplayInformation");
			}
			throw e;
		}
	}

	public static ReplayInformation decodeString(String serial) {
		try {
			ByteArrayInputStream inStr = new ByteArrayInputStream(serial.getBytes());
			ObjectInputStream in = new ObjectInputStream(inStr);
			ReplayInformation result = (ReplayInformation) in.readObject();
			return result;
		} catch (Exception e) {
			System.out.println("Error in deserialization");
			e.printStackTrace();
		}
		return null;

	}

}
