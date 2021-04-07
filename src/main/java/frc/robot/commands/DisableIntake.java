package frc.robot.commands;

import frc.robot.subsystems.IntakeStorageSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DisableIntake extends CommandBase {
	private final IntakeStorageSubsystem intakeStorageSubsystem;

	/**
	 * Creates a new RunShooterToRPMCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DisableIntake(IntakeStorageSubsystem intakeStorageSubsystem) {
		this.intakeStorageSubsystem = intakeStorageSubsystem;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intakeStorageSubsystem.retractIntake();
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
		return Math.abs(intakeStorageSubsystem.getIntakeMotorPower()) < 0.1;
	}
}
