/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;


public class DriveSubsystem extends SubsystemBase {
/**
* Creates a new ExampleSubsystem.
*/

CANSparkMax frontLeftMotor;
CANSparkMax backLeftMotor;
CANSparkMax frontRightMotor;
CANSparkMax backRightMotor;

SpeedControllerGroup leftSpeedControllerGroup;
SpeedControllerGroup rightSpeedControllerGroup;
DifferentialDrive differentialDrive;

AHRS gyro;

DifferentialDriveOdometry odometry;

public DriveSubsystem() {

frontLeftMotor = new CANSparkMax(Constants.SPARK_FRONT_LEFT_ID, MotorType.kBrushless);
backLeftMotor = new CANSparkMax(Constants.SPARK_BACK_LEFT_ID, MotorType.kBrushless);
frontRightMotor = new CANSparkMax(Constants.SPARK_FRONT_RIGHT_ID, MotorType.kBrushless);
backRightMotor = new CANSparkMax(Constants.SPARK_BACK_RIGHT_ID, MotorType.kBrushless);

leftSpeedControllerGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
rightSpeedControllerGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
differentialDrive = new DifferentialDrive(leftSpeedControllerGroup, rightSpeedControllerGroup);
gyro = new AHRS(SPI.Port.kMXP);
odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), new Pose2d(0, 0, new Rotation2d(0)));
initPID();
}

public void powerDrive(double leftPower, double rightPower) {
leftSpeedControllerGroup.set(leftPower);
rightSpeedControllerGroup.set(rightPower);
}

public void drivePID(double leftInches, double rightInches) {
double leftPosition = frontLeftMotor.getEncoder().getPosition();
double rightPosition = frontRightMotor.getEncoder().getPosition();

double leftTicks = (leftInches * Constants.DRIVE_TICKS_PER_INCH) + leftPosition;
double rightTicks = (rightInches * Constants.DRIVE_TICKS_PER_INCH) + rightPosition;

frontLeftMotor.getPIDController().setReference(leftTicks, ControlType.kPosition);
frontRightMotor.getPIDController().setReference(rightTicks, ControlType.kPosition);
}

private void setPID(CANSparkMax motor, double P, double I, double D) {
motor.getPIDController().setP(P);
motor.getPIDController().setI(I);
motor.getPIDController().setD(D);

}

private void initPID() {
setPID(frontLeftMotor, Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
setPID(frontRightMotor, Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
setPID(backLeftMotor, Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
setPID(backRightMotor, Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
}

public void tankDrive(){
differentialDrive.tankDrive(Robot.robotContainer.getLeftY(), Robot.robotContainer.getRightY(), true) ;

}

public void arcadeDrive() {
differentialDrive.arcadeDrive(Robot.robotContainer.getZ(), -Robot.robotContainer.getX());
}

public void stop() {
differentialDrive.stopMotor();
}

public void stopMotors() {
frontLeftMotor.set(0);
backLeftMotor.set(0);
frontRightMotor.set(0);
backRightMotor.set(0);
}

public double getHeading() {
return -Math.IEEEremainder(gyro.getAngle(), 360);
}

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
double leftRate = -frontLeftMotor.getEncoder().getVelocity() * Constants.DRIVE_METERS_PER_ROTATION / 60.0;
double rightRate = frontRightMotor.getEncoder().getVelocity() * Constants.DRIVE_METERS_PER_ROTATION / 60.0;
return new DifferentialDriveWheelSpeeds(leftRate, rightRate);
}

public Pose2d getPose() {
return odometry.getPoseMeters();
}

public void tankDriveVOlts(double leftVolts, double rightVolts) {
frontLeftMotor.setVoltage(leftVolts);
backLeftMotor.setVoltage(leftVolts);
frontRightMotor.setVoltage(-rightVolts);
backRightMotor.setVoltage(-rightVolts);
differentialDrive.feed();
}

public void resetOdometry(Pose2d pose) {
resetEncoders();
odometry.resetPosition(pose, gyro.getRotation2d());
}

public void resetEncoders() {
frontLeftMotor.getEncoder().setPosition(0);
frontRightMotor.getEncoder().setPosition(0);
}

public double getAverageEncoderDistance() {
return (frontLeftMotor.getEncoder().getPosition() + frontRightMotor.getEncoder().getPosition()) / 2.0;
}

public void resetGyro() {
gyro.reset();
}

public double getLeftEncoderPosition() {
return frontLeftMotor.getEncoder().getPosition();
}

public double getRightEncoderPosition() {
return frontRightMotor.getEncoder().getPosition();
}

public void setMaxOutput(double maxOutput) {
differentialDrive.setMaxOutput(maxOutput);
}

public double getTurnRate() {
//gyro defines clockwise as positive, when standard convention describes clockwise as negative
return -gyro.getRate();
}

@Override
public void periodic() {

// This method will be called once per scheduler run

odometry.update(gyro.getRotation2d(), frontLeftMotor.getEncoder().getPosition(), frontRightMotor.getEncoder().getPosition());
}
}
