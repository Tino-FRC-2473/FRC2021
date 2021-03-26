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
import frc.robot.commands.ChangeShooterPowerCommand;
import frc.robot.commands.DisableIntake;
import frc.robot.commands.EnableIntakeShooterCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;
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

	private final EnableIntakeShooterCommand enableIntakeShooterCommand = new EnableIntakeShooterCommand(shooterSubsystem, intakeStorageSubsystem, true); // for auto
	private final DisableIntake disableIntakeCommand = new DisableIntake(shooterSubsystem, intakeStorageSubsystem);
	private final Command autonomousCommand = 
		new ParallelCommandGroup (
			new EnableIntakeShooterCommand(shooterSubsystem, intakeStorageSubsystem, false),
			new SequentialCommandGroup (
				new StraightDrive(driveSubsystem, 144, 0.3),
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
	private JoystickButton leftBumper;
	private JoystickButton rightBumper;

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
		buttonA = new JoystickButton(driver, Constants.BUTTON_A); // A - runs shooter wheels + intake back up
		buttonB = new JoystickButton(driver, Constants.BUTTON_B); // B - stops shooter wheels
		buttonX = new JoystickButton(driver, Constants.BUTTON_X); // X - runs intake system
		buttonY = new JoystickButton(driver, Constants.BUTTON_Y); // Y - stops intake system
		leftBumper = new JoystickButton(driver, Constants.LEFT_BUMPER); // left bumper - decreases speed/power
		rightBumper = new JoystickButton(driver, Constants.RIGHT_BUMPER); // right bumper - increases speed/power

		// Arcade Drive
		wheel = new Joystick(Constants.WHEEL_PORT);
		throttle = new Joystick(Constants.THROTTLE_PORT);
		buttonPanel = new Joystick(Constants.BUTTON_PANEL_PORT);


		buttonA.whenPressed(new RunShooterCommand(shooterSubsystem, intakeStorageSubsystem, true));
		buttonB.whenPressed(new RunShooterCommand(shooterSubsystem, intakeStorageSubsystem, false));

		buttonX.whenPressed(new RunIntakeCommand(intakeStorageSubsystem, true));
		buttonY.whenPressed(new RunIntakeCommand(intakeStorageSubsystem, false));


		// shooter power testing
		leftBumper.whenPressed(new ChangeShooterPowerCommand(shooterSubsystem, -0.05));
		rightBumper.whenPressed(new ChangeShooterPowerCommand(shooterSubsystem, 0.05));
		

	}

	public EnableIntakeShooterCommand getEnableIntakeShooterCommand() {
		return enableIntakeShooterCommand;
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

	public Command disableIntake() {
		return disableIntakeCommand;
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
