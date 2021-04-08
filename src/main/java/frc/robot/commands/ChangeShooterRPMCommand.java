package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ChangeShooterRPMCommand extends CommandBase {

	private final ShooterSubsystem shooterSubsystem;
	private double velocityChange;

	/**
	 * Creates a new ChangeShooterPowerCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ChangeShooterRPMCommand(ShooterSubsystem subsystem, double velocity) {
		shooterSubsystem = subsystem;
		velocityChange = velocity;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shooterSubsystem.setTargetVelocity(shooterSubsystem.getVelocity() + velocityChange);
		System.out.println("Running speed to " + shooterSubsystem.getTargetVelocity());
		shooterSubsystem.runShooterRPM(shooterSubsystem.getTargetVelocity());
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
		if (Math.abs(shooterSubsystem.getVelocity() - shooterSubsystem.getTargetVelocity()) < 25) {
			if (velocityChange > 0) {
				System.out.println("Increased speed to " + shooterSubsystem.getTargetVelocity());
			} else {
				System.out.println("Decreased speed to " + shooterSubsystem.getTargetVelocity());
			}
			System.out.println();
			return true;
		} else {
			return false;
		}
	}

}