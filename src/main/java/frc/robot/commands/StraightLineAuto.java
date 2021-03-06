package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLineAuto extends CommandBase {

    DriveSubsystem driveSubsystem;
    private boolean isFinished = false;


    public StraightLineAuto(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.stopMotors();
    }

    @Override
    public void execute() {
       
        //drive forward 170 inches
        //turn left 90 degrees (to 45 deg position)
        //drive forward 84.9 inches
        //turn right 45 degrees (to 0 deg position)
        //drive forward 36 inches
        //due to space constraints,this command will only drive forward 36 inches
        //and not the entire 120 inches that the challenge requires
        System.out.println("StraightLineAuto command - starting execution");

        new SequentialCommandGroup (
            new StraightDrive(driveSubsystem, 180, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0),
            new StraightDrive(driveSubsystem, 36, 0.6)
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
