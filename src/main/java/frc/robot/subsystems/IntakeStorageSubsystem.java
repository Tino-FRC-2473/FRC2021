
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeStorageConstants;

public class IntakeStorageSubsystem extends SubsystemBase {

	private CANSparkMax intakeMotor;
	private CANSparkMax leftBeltMotor;
	private CANSparkMax rightBeltMotor;

	private DoubleSolenoid intakePistons;

 	/** Creates a new IntakeSubsystem. */
	public IntakeStorageSubsystem() {

		intakeMotor = new CANSparkMax(IntakeStorageConstants.SPARK_INTAKE, MotorType.kBrushless);

		// missing device IDs (0 is placeholder)
		leftBeltMotor = new CANSparkMax(0, MotorType.kBrushless);
		rightBeltMotor = new CANSparkMax(0, MotorType.kBrushless);

		intakePistons = new DoubleSolenoid(IntakeStorageConstants.INTAKE_PISTON_FORWARD_PORT, IntakeStorageConstants.INTAKE_PISTON_REVERSE_PORT);
	
	}

	public void runIntakeMotor(double power) {
		intakeMotor.set(power);
	}

	public void extendIntakePistons() {
		intakePistons.set(Value.kOff);
		intakePistons.set(Value.kForward);
	}

	public void retractIntakePistons() {
		intakePistons.set(Value.kOff);
		intakePistons.set(Value.kReverse);
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