// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.StraightLineAuto;
import frc.robot.commands.EnableIntake;
import frc.robot.commands.StraightDrive;
import frc.robot.commands.TurnUsingGyro;
import frc.robot.commands.DisableIntake;
import frc.robot.cv.CVDriveCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeStorageSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final IntakeStorageSubsystem intakeStorageSubsystem = new IntakeStorageSubsystem();

	private SequentialCommandGroup slalomPath =
		new SequentialCommandGroup (
			new TurnUsingGyro(driveSubsystem, 45),
			new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 0),
            new StraightDrive(driveSubsystem, 120, 0.6),
            new TurnUsingGyro(driveSubsystem, -45),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 45),
			new StraightDrive(driveSubsystem, 42.4, 0.6),
            new TurnUsingGyro(driveSubsystem, 135),
            new StraightDrive(driveSubsystem, 42.4, 0.6),
            new TurnUsingGyro(driveSubsystem, -135),
            new StraightDrive(driveSubsystem, 84.9, 0.6),
            new TurnUsingGyro(driveSubsystem, 180),
			new StraightDrive(driveSubsystem, 120, 0.6),
            new TurnUsingGyro(driveSubsystem, 135),
            new StraightDrive(driveSubsystem, 84.9, 0.6)
		);
	private SequentialCommandGroup bouncePath = 
		new SequentialCommandGroup (
			new TurnUsingGyro(driveSubsystem, 63.4),
			new StraightDrive(driveSubsystem, 67.1, 0.6),
            new TurnUsingGyro(driveSubsystem, -63.4),
            new StraightDrive(driveSubsystem, 134.2, 0.6),
            new TurnUsingGyro(driveSubsystem, 76.0),
            new StraightDrive(driveSubsystem, 123.7, 0.6),
            new TurnUsingGyro(driveSubsystem, -76.0),
			new StraightDrive(driveSubsystem, 123.7, 0.6),
            new TurnUsingGyro(driveSubsystem, 0),
            new StraightDrive(driveSubsystem, 30, 0.6),
            new TurnUsingGyro(driveSubsystem, 76.0),
            new StraightDrive(driveSubsystem, 123.7, 0.6),
            new TurnUsingGyro(driveSubsystem, -63.4),
			new StraightDrive(driveSubsystem, 67.1, 0.6),
            new TurnUsingGyro(driveSubsystem, 0)
		);
	private SequentialCommandGroup barrelPath = 
		new SequentialCommandGroup (
			new StraightDrive(driveSubsystem, 114, 0.6),
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new StraightDrive(driveSubsystem, 198, 0.6), 
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new TurnUsingGyro(driveSubsystem, -90),
			new StraightDrive(driveSubsystem, 120, 0.6),
			new TurnUsingGyro(driveSubsystem, 90),
			new StraightDrive(driveSubsystem, 60, 0.6),
			new TurnUsingGyro(driveSubsystem, 90),
			new StraightDrive(driveSubsystem, 60, 0.6), 
			new TurnUsingGyro(driveSubsystem, -90), 
			new StraightDrive(driveSubsystem, 252, 0.6)
		);


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 */

	// Tank Drive
	private XboxController driver;
	private Joystick joystick;
	private JoystickButton buttonA;
	private JoystickButton buttonB;
	private JoystickButton buttonX;
	private JoystickButton buttonY;

	// Arcade Drive
	private Joystick wheel;
	private Joystick throttle;
	private Joystick buttonPanel;
	public RobotContainer() {
		// Configure the button bindings
		System.out.println("Creating the auto command in robot container");
		configureButtonBindings();
	}

	private Command autonomousCommand;	


	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		driver = new XboxController(Constants.XBOX_CONTROLLER_PORT);
		// Tank Drive
		joystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
		buttonA = new JoystickButton(driver, Constants.BUTTON_A);
		buttonB = new JoystickButton(driver, Constants.BUTTON_B);
		buttonX = new JoystickButton(driver, Constants.BUTTON_X);
		buttonY = new JoystickButton(driver, Constants.BUTTON_Y);

		// Arcade Drive
		wheel = new Joystick(Constants.WHEEL_PORT);
		throttle = new Joystick(Constants.THROTTLE_PORT);
		buttonPanel = new Joystick(Constants.BUTTON_PANEL_PORT);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand(boolean isRedPath, boolean isPathA) {
		if(isRedPath && isPathA) {
			//red path a
			autonomousCommand = new SequentialCommandGroup(
				new EnableIntake(intakeStorageSubsystem),
				 new StraightDrive(driveSubsystem, 80, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, -26.6),
				 new StraightDrive(driveSubsystem, 67.1, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 71.6),
				 new StraightDrive(driveSubsystem, 94.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 0),
				 new StraightDrive(driveSubsystem, 186, Constants.AUTO_STRAIGHT_DRIVE_POWER));
		}else if(isRedPath && !isPathA) {
			//red path b
			autonomousCommand = new SequentialCommandGroup(
				new EnableIntake(intakeStorageSubsystem),
				 new StraightDrive(driveSubsystem, 80, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, -45),
				 new StraightDrive(driveSubsystem, 84.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 45),
				 new StraightDrive(driveSubsystem, 84.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 0),
				 new StraightDrive(driveSubsystem, 156, Constants.AUTO_STRAIGHT_DRIVE_POWER));
		}else if(!isRedPath && isPathA) {
			//blue path a
			autonomousCommand = new SequentialCommandGroup(
				new EnableIntake(intakeStorageSubsystem),
				 new StraightDrive(driveSubsystem, 176.0, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 71.6),
				 new StraightDrive(driveSubsystem, 94.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, -26.6),
				 new StraightDrive(driveSubsystem, 67.1, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 0),
				 new StraightDrive(driveSubsystem, 66, Constants.AUTO_STRAIGHT_DRIVE_POWER));
		}else {
			//blue path b
			autonomousCommand = new SequentialCommandGroup(
				new EnableIntake(intakeStorageSubsystem),
				 new StraightDrive(driveSubsystem, 170.0, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 45),
				 new StraightDrive(driveSubsystem, 84.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, -45),
				 new StraightDrive(driveSubsystem, 84.9, Constants.AUTO_STRAIGHT_DRIVE_POWER),
				 new TurnUsingGyro(driveSubsystem, 0),
				 new StraightDrive(driveSubsystem, 66, Constants.AUTO_STRAIGHT_DRIVE_POWER));
		}

		return autonomousCommand; 
	}

	public Command getDisableRobotCommand() {
		return new DisableIntake(intakeStorageSubsystem);
	}

	// public XboxController getDriver() {
	// 	return driver;
	// }

	// public DriveSubsystem getDriveSystem() {
	// 	return driveSubsystem;
	// }

	// public Joystick getLeftJoystick() {
	// 	return leftJoystick;
	// }

	// public Joystick getRightJoystick() {
	// 	return rightJoystick;
	// }

	public Joystick getWheel() {
		return wheel;
	}

	public Joystick getThrottle() {
		return throttle;
	}

	public Joystick getButtonPanel() {
		return buttonPanel;
	}

	public JoystickButton getButtonA() {
		return buttonA;
	}

	public JoystickButton getButtonB() {
		return buttonB;
	}

	public JoystickButton getButtonX() {
		return buttonX;
	}

	public JoystickButton getButtonY() {
		return buttonY;
	}

	public double getLeftY() {
		return driver.getY(GenericHID.Hand.kLeft);
	}

	public double getRightX() {
		return driver.getX(GenericHID.Hand.kRight);
	}

	public double getRightY() {
		return driver.getY(GenericHID.Hand.kRight);
	}

	public double getZ() {
		return getThrottle().getZ();
	}

	public double getX() {
		return getWheel().getX();
	}
}
