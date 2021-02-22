package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class StraightDrive extends CommandBase {

    DriveSubsystem driveSubsystem;
    private double heading;
    private boolean isFinished = false;
    private double power;
    private double endPosition;


    public StraightDrive(DriveSubsystem subsystem, double inches, double power) {
        driveSubsystem = subsystem;
        heading = driveSubsystem.getHeading();
        this.power = power;
        endPosition = inches * Constants.DRIVE_TICKS_PER_INCH + driveSubsystem.getAverageEncoderDistance();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() { 
        driveSubsystem.stopMotors();
    }

    @Override
    public void execute() {
        if(driveSubsystem.gyroDrive(endPosition, heading, power)) {
            isFinished = false;
        }else {
            isFinished = true;
        }   
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
