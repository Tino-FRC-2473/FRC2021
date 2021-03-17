// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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

    // units: meters 
    public static final double K_S_DRIVE = 0.126;
    public static final double K_V_DRIVE = 5.83;
    public static final double K_A_DRIVE = 0.88;
    public static final double K_P_DRIVE = 1.5;

    // placeholder trackwidth, need to update with empirical trackwidth after drive characterization
    public static final double K_TRACKWIDTH_METERS = 1.0565;
    public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(K_TRACKWIDTH_METERS);

    // units: m/s and m/s^2
    public static final double K_MAX_SPEED = 1.5;
    public static final double K_MAX_ACCEL = 3;

    // units: meters & seconds
    public static final double K_RAMSETE_B = 2;
    public static final double K_RAMSETE_ZETA = 0.7;

    public static final double DRIVE_P = 5e-4;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;

    public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
    public static final double GEAR_RATIO = 26.0 * 4.67 / 12.0;
    public static final double WHEEL_DIAMETER_INCHES = 7.65;
    public static final double DRIVE_TICKS_PER_INCH = COUNTS_PER_MOTOR_REVOLUTION * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES);
    public static final double DRIVE_WHEEL_CIRCUMFERENCE_INCHES = 23.8125;
    public static final double DRIVE_METERS_PER_ROTATION = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE_INCHES / GEAR_RATIO);

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
}
