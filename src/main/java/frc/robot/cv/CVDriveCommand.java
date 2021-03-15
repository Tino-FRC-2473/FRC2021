package frc.robot.cv;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;

public class CVDriveCommand extends CommandBase {

    DriveSubsystem driveSubsystem;
    CVData cvData;
  


    public CVDriveCommand(DriveSubsystem subsystem) {

        addRequirements(subsystem);
        
    }

    @Override
    public void initialize() { 
        cvData = Robot.getCVData();
    }

    @Override
    public void execute() {
        //currently just prints out the detected path
       System.out.println((cvData.isRedPath() ? "Red " : "Blue ") + (cvData.isPathA() ? "A" : "B"));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
