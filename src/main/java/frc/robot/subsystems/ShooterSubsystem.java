
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

	private CANSparkMax leftShooterMotor;
	private CANSparkMax rightShooterMotor;
	private double targetPower;
	private double targetVelocity;

 	/** Creates a new ShooterSubsystem. */
	public ShooterSubsystem() {

		leftShooterMotor = new CANSparkMax(ShooterConstants.SPARK_SHOOTER_LEFT, MotorType.kBrushless);
		rightShooterMotor = new CANSparkMax(ShooterConstants.SPARK_SHOOTER_RIGHT, MotorType.kBrushless);

		targetPower = 0.4;
		targetVelocity = 5900;
		leftShooterMotor.set(0); // on init, set power to 0
		rightShooterMotor.set(0);

	}

	public void runShooterPower(double power) {
		leftShooterMotor.set(power);
		rightShooterMotor.set(-power);
	}

	public void runShooterRPM(double rpm) { // 5834
		leftShooterMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
		rightShooterMotor.getPIDController().setReference(-rpm, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
    	// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
    	// This method will be called once per scheduler run during simulation
	}

	public double getPower() {
		return leftShooterMotor.get();
	}

	public double getVelocity() {
		return leftShooterMotor.getEncoder().getVelocity();
	}

	public double getTargetPower() {
		return targetPower;
	}

	public void setTargetPower(double targetPower) {
		this.targetPower = targetPower;
	}

	public double getTargetVelocity() {
		return targetVelocity;
	}

	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
	}
}