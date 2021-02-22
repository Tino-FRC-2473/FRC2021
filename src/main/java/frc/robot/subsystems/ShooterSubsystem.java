
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

	private CANSparkMax leftShooterMotor;
	private CANSparkMax rightShooterMotor;

 	/** Creates a new ShooterSubsystem. */
	public ShooterSubsystem() {

		leftShooterMotor = new CANSparkMax(ShooterConstants.SPARK_SHOOTER_LEFT, MotorType.kBrushless);
		rightShooterMotor = new CANSparkMax(ShooterConstants.SPARK_SHOOTER_RIGHT, MotorType.kBrushless);

	}

	public void runShooterPower(double power) {
		leftShooterMotor.set(power);
		rightShooterMotor.set(-power);
	}

	@Override
	public void periodic() {
    	// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
    	// This method will be called once per scheduler run during simulation
	}
}