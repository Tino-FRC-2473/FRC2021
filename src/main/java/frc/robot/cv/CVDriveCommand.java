package frc.robot.cv;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;

public class CVDriveCommand extends SequentialCommandGroup {

    DriveSubsystem driveSubsystem;
    CVData cvData;
    SequentialCommandGroup path;
    private boolean isFinished = false;
    SequentialCommandGroup RedPathA;
    SequentialCommandGroup RedPathB;
    SequentialCommandGroup BluePathA ;
    SequentialCommandGroup BluePathB;
  


    public CVDriveCommand(DriveSubsystem subsystem) {

        driveSubsystem = subsystem;
        addRequirements(subsystem);

        RedPathA = 
        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 60, 0.6),
            new TurnUsingGyro(driveSubsystem, -26.6),
            new StraightDrive(driveSubsystem, 67.1, 0.6),
            new TurnUsingGyro(driveSubsystem, 71.6),
            new StraightDrive(driveSubsystem, 94.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
        RedPathB = 
        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 60, 0.6),
            new TurnUsingGyro(driveSubsystem, -45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
        BluePathA = 
        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 150.0, 0.6),
            new TurnUsingGyro(driveSubsystem, 71.6),
            new StraightDrive(driveSubsystem, 94.9, 0.6),
            new TurnUsingGyro(driveSubsystem, -26.6),
            new StraightDrive(driveSubsystem, 67.1, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
        //new TurnUsingGyro(driveSubsystem, -21.8),

        BluePathB = 
        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 120.0, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, -45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
        );
        //new TurnUsingGyro(driveSubsystem, -21.8),
        
    }

    @Override
    public void initialize() { 
        cvData = Robot.getCVData();
        System.out.println("Path: " + (cvData.isRedPath() ? "Red " : "Blue ") + (cvData.isPathA() ? "A" : "B"));
       if(cvData.isRedPath() && cvData.isPathA()) {
           //red path a
            addCommands(
                new StraightDrive(driveSubsystem, 60, 0.6),
                new TurnUsingGyro(driveSubsystem, -26.6),
                new StraightDrive(driveSubsystem, 67.1, 0.6),
                new TurnUsingGyro(driveSubsystem, 71.6),
                new StraightDrive(driveSubsystem, 94.9, 0.6),
                new TurnUsingGyro(driveSubsystem, 0));
       }else if(cvData.isRedPath() && !cvData.isPathA()) {
           //red path b
            addCommands(
                new StraightDrive(driveSubsystem, 60, 0.6),
                new TurnUsingGyro(driveSubsystem, -45),
                new StraightDrive(driveSubsystem, 84.9, 0.6),
                new TurnUsingGyro(driveSubsystem, 45),
                new StraightDrive(driveSubsystem, 84.9, 0.6),
                new TurnUsingGyro(driveSubsystem, 0));
       }else if(!cvData.isRedPath() && cvData.isPathA()) {
           //blue path a
            addCommands(
                new StraightDrive(driveSubsystem, 150.0, 0.6),
                new TurnUsingGyro(driveSubsystem, 71.6),
                new StraightDrive(driveSubsystem, 94.9, 0.6),
                new TurnUsingGyro(driveSubsystem, -26.6),
                new StraightDrive(driveSubsystem, 67.1, 0.6),
                new TurnUsingGyro(driveSubsystem, 0));
       }else {
           //blue path b
            addCommands(
                new StraightDrive(driveSubsystem, 120.0, 0.6),
                new TurnUsingGyro(driveSubsystem, 45),
                new StraightDrive(driveSubsystem, 84.9, 0.6),
                new TurnUsingGyro(driveSubsystem, -45),
                new StraightDrive(driveSubsystem, 84.9, 0.6),
                new TurnUsingGyro(driveSubsystem, 0));
       }
       System.out.println("Path added");
    }

    // @Override
    // public void execute() {
       
    // }

    // @Override
    // public void end(boolean interrupted) {
    // }

    // @Override
    // public boolean isFinished() {
    //     return this.isFinished;
    // }
}
