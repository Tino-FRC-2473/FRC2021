// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeStorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeStorageSubsystem intakeStorageSubsystem;
  private boolean status;

  /**
   * Creates a new RunIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCommand(IntakeStorageSubsystem intakeStorageSubsystem, boolean status) {
    this.intakeStorageSubsystem = intakeStorageSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.status = status;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (status) {
      intakeStorageSubsystem.deployIntake(0.5);
    } else {
      intakeStorageSubsystem.retractIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (status) {
			return Math.abs(intakeStorageSubsystem.getIntakeMotorPower() - intakeStorageSubsystem.getTargetIntakeMotorPower()) < 0.01;
		}
		return Math.abs(intakeStorageSubsystem.getIntakeMotorPower()) < 0.01;
  }
}