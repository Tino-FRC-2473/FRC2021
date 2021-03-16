package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeStorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class EnableIntakeShooterCommand extends CommandBase {

	private final ShooterSubsystem shooterSubsystem;
	private final IntakeStorageSubsystem intakeStorageSubsystem;
	private boolean status;

	/**
	 * Creates a new RunShooterToRPMCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public EnableIntakeShooterCommand(ShooterSubsystem shooterSubsystem, IntakeStorageSubsystem intakeStorageSubsystem, boolean status) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeStorageSubsystem = intakeStorageSubsystem;
		this.status = status;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (status) {
			shooterSubsystem.runShooterPower(shooterSubsystem.getTargetPower());
		} else {
			shooterSubsystem.runShooterPower(0);
		}
		intakeStorageSubsystem.deployIntake(0.5);
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
		if (status) {
			return Math.abs(shooterSubsystem.getPower() - shooterSubsystem.getTargetPower()) < 0.01;
		}
		return Math.abs(shooterSubsystem.getPower()) < 0.01;
	}

}