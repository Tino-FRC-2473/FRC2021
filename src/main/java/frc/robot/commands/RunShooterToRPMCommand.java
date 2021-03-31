package frc.robot.commands;

import frc.robot.subsystems.IntakeStorageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunShooterToRPMCommand extends CommandBase {

	private final ShooterSubsystem shooterSubsystem;
	private final IntakeStorageSubsystem intakeStorageSubsystem;
	private boolean status;
	private Timer intakeTimer;

	/**
	 * Creates a new RunShooterToRPMCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public RunShooterToRPMCommand(ShooterSubsystem shooterSubsystem, IntakeStorageSubsystem intakeStorageSubsystem, boolean status) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeStorageSubsystem = intakeStorageSubsystem;
		this.status = status;
		intakeTimer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (status) {
			intakeTimer.reset();
			intakeTimer.start();
			intakeStorageSubsystem.runStorageMotor(-intakeStorageSubsystem.getTargetIntakeMotorPower());
		} else {
			shooterSubsystem.runShooterRPM(0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (intakeTimer.hasElapsed(0.5)) {
			intakeStorageSubsystem.runStorageMotor(0);
			shooterSubsystem.runShooterRPM(shooterSubsystem.getTargetVelocity());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (status) {
			if (Math.abs(shooterSubsystem.getVelocity() - shooterSubsystem.getTargetVelocity()) < 25) {
				intakeStorageSubsystem.runStorageMotor(intakeStorageSubsystem.getTargetIntakeMotorPower());
				return true;
			} else {
				return false;
			}
		} else {
			return Math.abs(shooterSubsystem.getVelocity()) < 25;
		}
	}

}