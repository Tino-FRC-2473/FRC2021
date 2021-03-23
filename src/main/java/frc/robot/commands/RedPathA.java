package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedPathA extends CommandBase {

    DriveSubsystem driveSubsystem;
    private boolean isFinished = false;


    public RedPathA(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stopMotors();
    }

    @Override
    public void execute() {

        System.out.println("Running Red Path A");

        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 60, 0.6),
            new TurnUsingGyro(driveSubsystem, -26.6),
            new StraightDrive(driveSubsystem, 67.1, 0.6),
            new TurnUsingGyro(driveSubsystem, 71.6),
            new StraightDrive(driveSubsystem, 94.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
        isFinished = true;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }
}
