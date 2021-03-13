// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeStorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeStorageSubsystem intake_subsystem;

  /**
   * Creates a new RunIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCommand(IntakeStorageSubsystem subsystem) {
    intake_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void runIntake(){
    intake_subsystem.runIntakeMotor(0.75);
  }

  public void runStorage(){
    intake_subsystem.runStorageMotor(0.75);
  }
}