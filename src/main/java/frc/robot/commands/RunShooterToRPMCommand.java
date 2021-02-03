package frc.robot.commands;

import frc.robot.subsystems.ShooterPrototypeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunShooterToRPMCommand extends CommandBase {
  
  private final ShooterPrototypeSubsystem shooterSubsystem;
  private double power;

  /**
   * Creates a new RunShooterToRPMCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunShooterToRPMCommand(ShooterPrototypeSubsystem subsystem, double power) {
    shooterSubsystem = subsystem;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooterSubsystem.runShooterPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
