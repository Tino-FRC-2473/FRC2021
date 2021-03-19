// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cv;

import edu.wpi.first.wpilibj.SerialPort;

public class Jetson extends SerialPort {

	private String info = "";
	private boolean isRedPath = true;
	private boolean isPathA = true;

	public Jetson(int baudRate, Port port) {
		super(baudRate, port);
	}

	public void updateVision() {
		info = readString();
		System.out.println("String from the Jetson: " + info);
		//make sure the string is of correct length
		if(info.indexOf("S") >= 0) {
			int startIndex = info.indexOf("S");
			//check is the path is the red or blue path
			if(info.substring( startIndex + 2, startIndex + 3).equalsIgnoreCase("R")) {
				isRedPath = true;
			}else {
				isRedPath = false;
			}
			//check is the path is path A or path B
			if(info.substring(startIndex + 3, startIndex + 4).equalsIgnoreCase("A")) {
				isPathA = true;
			}else {
				isPathA = false;
			}
		}
	}

	public CVData getCVData() {
		//not sure if it is necessary to update the vision here if I am also
		//updating it on autonomous init
		updateVision();
		return new CVData(isRedPath, isPathA);
	}
}
