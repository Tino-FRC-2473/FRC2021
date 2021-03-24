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
  


    public CVDriveCommand(DriveSubsystem subsystem) {

        addRequirements(subsystem);
        
    }

    @Override
    public void initialize() { 
        cvData = Robot.getCVData();
        System.out.println("Path: " + (cvData.isRedPath() ? "Red " : "Blue ") + (cvData.isPathA() ? "A" : "B"));
       if(cvData.isRedPath() && cvData.isPathA()) {
            path = new RedPathA(driveSubsystem);
       }else if(cvData.isRedPath() && !cvData.isPathA()) {
            path = new RedPathB(driveSubsystem);
       }else if(!cvData.isRedPath() && cvData.isPathA()) {
            path = new BluePathA(driveSubsystem);
       }else {
            path = new BluePathB(driveSubsystem);
       }
       addCommands(path);
    }

    @Override
    public void execute() {
       isFinished = path.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }
}
