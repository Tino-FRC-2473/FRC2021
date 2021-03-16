
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
	private CANSparkMax storageMotor;
	private double targetIntakeMotorPower;

	private DoubleSolenoid intakePistons;

 	/** Creates a new IntakeSubsystem. */
	public IntakeStorageSubsystem() {

		intakeMotor = new CANSparkMax(IntakeStorageConstants.SPARK_INTAKE, MotorType.kBrushless);
		storageMotor = new CANSparkMax(IntakeStorageConstants.SPARK_STORAGE, MotorType.kBrushless);

		intakePistons = new DoubleSolenoid(IntakeStorageConstants.INTAKE_PISTON_FORWARD_PORT, IntakeStorageConstants.INTAKE_PISTON_REVERSE_PORT);
	
		targetIntakeMotorPower = 0.5;


		intakeMotor.set(0);
		storageMotor.set(0);

		// intakePistons.set(Value.kOff);


	}

	public void runIntakeMotor(double power) {
		intakeMotor.set(power);
	}

	public void deployIntake(double power) {
		extendIntakePistons();
		runIntakeMotor(power);
		runStorageMotor(power);
	}

	public void retractIntake() {
		retractIntakePistons();
		runIntakeMotor(0);
	}

	public void extendIntakePistons() {
		intakePistons.set(Value.kOff);
		intakePistons.set(Value.kForward);
	}

	public void retractIntakePistons() {
		intakePistons.set(Value.kOff);
		intakePistons.set(Value.kReverse);
	}

	public void runStorageMotor(double power) {
		storageMotor.set(power);
	}

	public double getIntakeMotorPower() {
		return intakeMotor.get();
	}

	public double getTargetIntakeMotorPower() {
		return targetIntakeMotorPower;
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