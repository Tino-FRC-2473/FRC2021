package frc.robot.commands;

import frc.robot.subsystems.IntakeStorageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunShooterCommand extends CommandBase {

	private final ShooterSubsystem shooterSubsystem;
	private final IntakeStorageSubsystem intakeStorageSubsystem;
	private boolean status;
	private Timer intakeTimer;

	/**
	 * Creates a new RunShooterToRPMCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public RunShooterCommand(ShooterSubsystem shooterSubsystem, IntakeStorageSubsystem intakeStorageSubsystem, boolean status) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeStorageSubsystem = intakeStorageSubsystem;
		this.status = status;
		intakeTimer = new Timer();
		intakeTimer.reset();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (status) {
			intakeTimer.start();
			intakeStorageSubsystem.runStorageMotor(-intakeStorageSubsystem.getTargetIntakeMotorPower());
		} else {
			shooterSubsystem.runShooterPower(0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (intakeTimer.hasElapsed(2)) {
			intakeStorageSubsystem.runStorageMotor(0);
			shooterSubsystem.runShooterPower(shooterSubsystem.getTargetPower());
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
			if (Math.abs(shooterSubsystem.getPower() - shooterSubsystem.getTargetPower()) < 0.01) {
				intakeStorageSubsystem.runStorageMotor(intakeStorageSubsystem.getTargetIntakeMotorPower());
				return true;
			} else {
				return false;
			}
			
		} else {
			return Math.abs(shooterSubsystem.getPower()) < 0.01;
		}
	}

}