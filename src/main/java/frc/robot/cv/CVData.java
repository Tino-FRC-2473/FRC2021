// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cv;

import edu.wpi.first.wpilibj.SerialPort;

public class CVData {

	private boolean isRedPath;
	private boolean isPathA;

	public CVData(boolean isRedPath, boolean isPathA) {
		this.isRedPath = isRedPath;
		this.isPathA = isPathA;
	}

	public boolean isPathA() {
		return isPathA;
	}

	public boolean isRedPath() {
		return isRedPath;
	}
}
