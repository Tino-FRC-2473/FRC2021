/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

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
	// public final ServoSubsystem servoSubsystem = new ServoSubsystem();

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
		// create a voltage constraint to ensure we don't accelerate too fast
		// set maximum voltage to 10V rather than 12V to allow for voltage sag during operation
		DifferentialDriveVoltageConstraint autoVoltageConstraint = 
			new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.K_S_DRIVE, Constants.K_V_DRIVE, Constants.K_A_DRIVE),
			Constants.K_DRIVE_KINEMATICS,
			10);

		// create configuration for trajectory
		TrajectoryConfig config = 
			new TrajectoryConfig(Constants.K_MAX_SPEED, 
								 Constants.K_MAX_ACCEL)
								 // add kinematics to ensure max speed is obeyed
								 .setKinematics(Constants.K_DRIVE_KINEMATICS)
								 // apply voltage constraint
								 .addConstraint(autoVoltageConstraint);

		Pose2d initialPoseB1 = new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(10), new Rotation2d(0));	
		
		Translation2d B3 = new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(10));

		Translation2d D5 = new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(5));

		Translation2d B7 = new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(10));

		Pose2d testPoseD5 = new Pose2d(Units.feetToMeters(12.5), Units.feetToMeters(5), new Rotation2d(0));
		
		Pose2d endPoseB11 = new Pose2d(Units.feetToMeters(27), Units.feetToMeters(10), new Rotation2d(0));
		
		// Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
		// 	// starting position of robot
		// 	initialPoseB1,
		// 	// pass through these waypoints to intake power cells
		// 	List.of(B3, D5, B7),
		// 	// end at this position 
		// 	endPoseB11,
		// 	// pass config
		// 	config
		// );

		// trajectory for testing:
		Trajectory trajTest = TrajectoryGenerator.generateTrajectory(
			// starting position of robot
			initialPoseB1,
			// pass through these waypoints to intake power cells
			List.of(B3),
			// end at this position 
			testPoseD5,
			// pass config
			config
		);

		RamseteCommand ramseteCommand = new RamseteCommand( 
			trajTest,
			driveSubsystem::getPose,
			new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_ZETA),
			new SimpleMotorFeedforward(Constants.K_S_DRIVE, 
									   Constants.K_V_DRIVE, 
									   Constants.K_A_DRIVE),
			Constants.K_DRIVE_KINEMATICS,
			driveSubsystem::getWheelSpeeds,
			new PIDController(Constants.K_P_DRIVE, 0, 0),
			new PIDController(Constants.K_P_DRIVE, 0, 0),
			// RamseteCommand passes volts to the callback
			driveSubsystem::tankDriveVolts,
			driveSubsystem
		);

		// reset odometry to starting pose of trajectory
		driveSubsystem.resetOdometry(trajTest.getInitialPose());

		// run path following, then stop
		return ramseteCommand.andThen(()-> driveSubsystem.tankDriveVolts(0, 0));
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

	public double getLeftY() {
		return getLeftJoystick().getY();
	}

	public double getRightY() {
		return getRightJoystick().getY();
	}

	public double getZ() {
		return getThrottle().getZ();
	}

	public double getX() {
		return getWheel().getX();
	}
}
