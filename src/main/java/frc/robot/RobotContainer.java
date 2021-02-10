/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TestMotorCommand;
import frc.robot.subsystems.TestMotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ServoSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public final TestMotorSubsystem testMotorSubsystem = new TestMotorSubsystem();
	public final TestMotorCommand testMotorCommand = new TestMotorCommand(testMotorSubsystem);

	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final ServoSubsystem servoSubsystem = new ServoSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 */

	// Tank Drive
	private XboxController driver;
	private Joystick leftJoystick;
	private Joystick rightJoystick;
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
		leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
		rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
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
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous

		return testMotorCommand;
		// return testMotorEncoderCommand;
	}

	public XboxController getDriver() {
		return driver;
	}

	public DriveSubsystem getDriveSystem() {
		return driveSubsystem;
	}

	public Joystick getLeftJoystick() {
		return leftJoystick;
	}

	public Joystick getRightJoystick() {
		return rightJoystick;
	}

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

	public float getLeftY() {
		return getLeftJoystick().getY();
	}

	public float getRightY() {
		return getRightJoystick().getY();
	}

	public float getZ() {
		return getThrottle().getZ();
	}

	public float getX() {
		return getWheel().getX();
	}
}
