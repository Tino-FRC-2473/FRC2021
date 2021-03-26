package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ChangeShooterPowerCommand extends CommandBase {

	private final ShooterSubsystem shooterSubsystem;
	private double powerChange;

	/**
	 * Creates a new ChangeShooterPowerCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ChangeShooterPowerCommand(ShooterSubsystem subsystem, double power) {
		shooterSubsystem = subsystem;
		powerChange = power;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shooterSubsystem.setTargetPower(shooterSubsystem.getPower() + powerChange);
		System.out.println("Running power to " + shooterSubsystem.getTargetPower());
		shooterSubsystem.runShooterPower(shooterSubsystem.getTargetPower());
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
		if (Math.abs(shooterSubsystem.getPower() - shooterSubsystem.getTargetPower()) < 0.01) {
			if (powerChange > 0) {
				System.out.println("Increased power to " + shooterSubsystem.getTargetPower());
				System.out.println("Current velocity: " + shooterSubsystem.getVelocity());
			} else {
				System.out.println("Decreased power to " + shooterSubsystem.getTargetPower());
				System.out.println("Current velocity: " + shooterSubsystem.getVelocity());
			}
			System.out.println();
			return true;
		} else {
			return false;
		}
	}

}