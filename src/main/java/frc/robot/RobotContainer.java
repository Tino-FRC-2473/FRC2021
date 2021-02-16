// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterPrototypeSubsystem;
import frc.robot.commands.ChangeShooterPowerCommand;
import frc.robot.commands.EnableShooterCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GamepadConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a “declarative” paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// The robot’s subsystems and commands are defined here...
	private final ShooterPrototypeSubsystem shooterTestSubsystem = new ShooterPrototypeSubsystem();
	private XboxController gamepad;
	private JoystickButton gamepadButtonA;
	private JoystickButton gamepadButtonB;
	private JoystickButton gamepadButtonX;
	private JoystickButton gamepadButtonY;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
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

		gamepad = new XboxController(GamepadConstants.XBOX_CONTROLLER_PORT);

		gamepadButtonA = new JoystickButton(gamepad, GamepadConstants.BUTTON_A); // A - starts running the motor
		gamepadButtonB = new JoystickButton(gamepad, GamepadConstants.BUTTON_B); // B - stop running the motor
		gamepadButtonX = new JoystickButton(gamepad, GamepadConstants.BUTTON_X); // X - increase power by 0.1
		gamepadButtonY = new JoystickButton(gamepad, GamepadConstants.BUTTON_Y); // Y - decrease power by 0.1

		gamepadButtonA.whenPressed(new EnableShooterCommand(shooterTestSubsystem, true));
		gamepadButtonB.whenPressed(new EnableShooterCommand(shooterTestSubsystem, false));
		gamepadButtonX.whenPressed(new ChangeShooterPowerCommand(shooterTestSubsystem, 0.1));
		gamepadButtonY.whenPressed(new ChangeShooterPowerCommand(shooterTestSubsystem, -0.1));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}
}