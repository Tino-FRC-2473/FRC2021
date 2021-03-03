// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


// Commands
import frc.robot.commands.StraightLineAuto;
import frc.robot.commands.StraightDrive;
import frc.robot.commands.TurnUsingGyro;
import frc.robot.commands.EnableShooterCommand;

// Subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeStorageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	private final IntakeStorageSubsystem intakeStorageSubsystem = new IntakeStorageSubsystem();

	private final EnableShooterCommand enableShooterCommand = new EnableShooterCommand(shooterSubsystem, intakeStorageSubsystem, false);
	private final Command autonomousCommand = 
		new ParallelCommandGroup (
			new EnableShooterCommand(shooterSubsystem, intakeStorageSubsystem, false),
			new SequentialCommandGroup (
				new StraightDrive(driveSubsystem, 170, 0.3),
				new TurnUsingGyro(driveSubsystem, 90),
				new StraightDrive(driveSubsystem, 85, 0.3),
				new TurnUsingGyro(driveSubsystem, 45),
				new StraightDrive(driveSubsystem, 24, 0.3)
			)
		);
	// public final ServoSubsystem servoSubsystem = new ServoSubsystem();

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

	public EnableShooterCommand getEnableShooterCommand() {
		return enableShooterCommand;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous

		return autonomousCommand; 
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
