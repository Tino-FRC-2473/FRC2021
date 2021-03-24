package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedPathB extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;


    public RedPathB(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stopMotors();
        System.out.println("Running Red Path B");
        addCommands(    
            new StraightDrive(driveSubsystem, 60, 0.6),
            new TurnUsingGyro(driveSubsystem, -45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
    }

    // @Override
    // public void execute() {

    //     isFinished = path.isFinished();
        
    // }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return this.isFinished;
    // }
}
