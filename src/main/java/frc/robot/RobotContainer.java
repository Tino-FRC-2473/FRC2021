// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterPrototypeSubsystem;
import frc.robot.commands.RunShooterToRPMCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * “declarative” paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot’s subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final ShooterPrototypeSubsystem shooterTestSubsystem = new ShooterPrototypeSubsystem();
  private XboxController gamepad = new XboxController(1);
  private double shooterPower = 0.3;
  private JoystickButton gamepadButtonA = new JoystickButton(gamepad, 1);
  private JoystickButton gamepadButtonB = new JoystickButton(gamepad, 2);
  private JoystickButton gamepadButtonX = new JoystickButton(gamepad, 3);
  private JoystickButton gamepadButtonY = new JoystickButton(gamepad, 4);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // A - starts running the motor
    // B - stop running the motor
    // X - increase power by 0.1
    // Y - decrease power by 0.1
    gamepadButtonA.whenPressed(new RunShooterToRPMCommand(shooterTestSubsystem, shooterPower));
    gamepadButtonB.whenPressed(new RunShooterToRPMCommand(shooterTestSubsystem, 0));
    gamepadButtonX.whenPressed(
      new RunShooterToRPMCommand(shooterTestSubsystem, ++shooterPower));
    gamepadButtonY.whenPressed(
      new RunShooterToRPMCommand(shooterTestSubsystem, --shooterPower));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}