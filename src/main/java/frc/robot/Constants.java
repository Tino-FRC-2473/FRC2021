// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int TEST_PORT = 11;
    public static final double ENCODER_INCHES_TO_TICKS = 1;

    public static final int SPARK_FRONT_LEFT_ID = 3; 
    public static final int SPARK_BACK_LEFT_ID = 4; 
    public static final int SPARK_FRONT_RIGHT_ID = 1; 
    public static final int SPARK_BACK_RIGHT_ID = 2;  

    public static final double DRIVE_P = 5e-4;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;

    public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
    public static final double GEAR_RATIO = 26.0 * 4.67 / 12.0;
    public static final double WHEEL_DIAMETER_INCHES = 7.65;
    public static final double DRIVE_TICKS_PER_INCH = COUNTS_PER_MOTOR_REVOLUTION * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES);

	public static final int WHEEL_PORT = 0;
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 2;
    public static final int THROTTLE_PORT = 3;
    public static final int BUTTON_PANEL_PORT = 4;
    
    
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;

    public static final int SERVO_PORT = 0;

    public static final class IntakeStorageConstants {
		public static final int SPARK_INTAKE = 9;
		public static final int SPARK_STORAGE = 8;

		public static final int INTAKE_PISTON_FORWARD_PORT = 6;
		public static final int INTAKE_PISTON_REVERSE_PORT = 1;
    }
    
    public static final class ShooterConstants {
		public static final int SPARK_SHOOTER_LEFT = 6;
		public static final int SPARK_SHOOTER_RIGHT = 7;
    }
}
