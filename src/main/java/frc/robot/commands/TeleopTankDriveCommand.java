package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopTankDriveCommand extends CommandBase {

    DriveSubsystem driveSubsystem;


    public TeleopTankDriveCommand(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() { 
        driveSubsystem.stopMotors();
        driveSubsystem.resetOdometry(new Pose2d());
    }

    @Override
    public void execute() {
        driveSubsystem.tankDrive();
        System.out.println(driveSubsystem.getPose());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
