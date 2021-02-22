package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnUsingGyro extends CommandBase {

    DriveSubsystem driveSubsystem;
    private double targetAngleDeg;
    private double accuracy;
    private boolean isFinished = false;


    public TurnUsingGyro(DriveSubsystem subsystem, double targetAngleDeg, double accuracy) {
        driveSubsystem = subsystem;
        this.targetAngleDeg = targetAngleDeg;
        this.accuracy = accuracy;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() { 
        driveSubsystem.stopMotors();
    }

    @Override
    public void execute() {
        double error = targetAngleDeg - driveSubsystem.getHeading();
        if(Math.abs(error) >= accuracy) {
            driveSubsystem.gyroTurn(targetAngleDeg);
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
