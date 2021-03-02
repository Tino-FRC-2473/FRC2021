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
        this.power = power;
        endPosition = inches * Constants.DRIVE_TICKS_PER_INCH;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() { 
        //driveSubsystem.stopMotors();
        heading = driveSubsystem.getHeading();
        System.out.println("got the heading in straightdrive command");
        endPosition += driveSubsystem.getAverageEncoderDistance() * Constants.COUNTS_PER_MOTOR_REVOLUTION;
    }

    @Override
    public void execute() {
        System.out.println("Heading: " + heading + "  endPosition: " + endPosition);
        isFinished = driveSubsystem.gyroDrive(endPosition, heading, power);  
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
