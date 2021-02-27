// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeStorageSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousCommand extends CommandBase {
  
  private final IntakeStorageSubsystem intakeStorageSubsystem;
  private boolean isFinished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommand(IntakeStorageSubsystem subsystem) {
    intakeStorageSubsystem = subsystem;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeStorageSubsystem.deployIntake(0.5);
    intakeStorageSubsystem.runStorageMotor(0.5);
    isFinished = true;
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
    return isFinished;
  }
}
