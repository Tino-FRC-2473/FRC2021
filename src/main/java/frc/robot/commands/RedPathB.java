package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedPathB extends CommandBase {

    DriveSubsystem driveSubsystem;
    private boolean isFinished = false;


    public RedPathB(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stopMotors();
    }

    @Override
    public void execute() {

        System.out.println("Running Red Path B");

        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 60, 0.6),
            new TurnUsingGyro(driveSubsystem, -45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
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
