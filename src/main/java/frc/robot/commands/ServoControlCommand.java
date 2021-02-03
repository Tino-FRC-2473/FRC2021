/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ServoControlCommand extends CommandBase {
  /**
   * Creates a new Teleop.
   */
  public ServoControlCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.robotContainer.getButtonA().whenReleased(new RunServo(0, Robot.robotContainer.servoSubsystem));
    Robot.robotContainer.getButtonB().whenReleased(new RunServo(90, Robot.robotContainer.servoSubsystem));
    Robot.robotContainer.getButtonX().whenReleased(new RunServo(180, Robot.robotContainer.servoSubsystem));
    Robot.robotContainer.getButtonY().whenReleased(new RunServo(270, Robot.robotContainer.servoSubsystem));
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
