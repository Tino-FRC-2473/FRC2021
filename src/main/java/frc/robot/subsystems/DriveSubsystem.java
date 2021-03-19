// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

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

	public double accessLeftVolts, accessRightVolts;

	public DriveSubsystem() {

		frontLeftMotor = new CANSparkMax(Constants.SPARK_FRONT_LEFT_ID, MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(Constants.SPARK_BACK_LEFT_ID, MotorType.kBrushless);
		frontRightMotor = new CANSparkMax(Constants.SPARK_FRONT_RIGHT_ID, MotorType.kBrushless);  
		backRightMotor = new CANSparkMax(Constants.SPARK_BACK_RIGHT_ID, MotorType.kBrushless); 
        
        frontLeftMotor.getEncoder().setPosition(0);
        backLeftMotor.getEncoder().setPosition(0);
        frontRightMotor.getEncoder().setPosition(0);
        frontLeftMotor.getEncoder().setPosition(0);

		leftSpeedControllerGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor); 
		rightSpeedControllerGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);
		
		differentialDrive = new DifferentialDrive(leftSpeedControllerGroup, rightSpeedControllerGroup); 
		gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
        gyro.zeroYaw();
        System.out.println(gyro.getAngle());

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
                new Pose2d(0, 0, new Rotation2d(0)));

        setMaxOutput(0.75);
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

	public boolean gyroDrive(double endPositionTick, double heading, double power) {
        double currentHeading = getHeading();
		double error = heading - currentHeading;
        System.out.println("current Heading: " + currentHeading);
        double position = getAverageEncoderDistance() * Constants.COUNTS_PER_MOTOR_REVOLUTION;
		double adjustedLeftPower = power - error / 180;
		double adjustedRightPower = power + error / 180;
		double distanceToTarget = Math.abs(endPositionTick - position) / Constants.ENCODER_INCHES_TO_TICKS;
        System.out.println("error: " + error + "  leftPower: " + adjustedLeftPower + " rightPower:" + adjustedRightPower + "  distanceToTarget: " + distanceToTarget);
		//the scalar slows down the robot as it gets closer to the goal,
		//while ensuring that it doesn't slow the robot down too much (keeps scalar between 1 and 0.1)
		double scalar = 1; //Math.min(1, Math.max(Math.atan(distanceToTarget / 4) * 2 / Math.PI, 0.1));
		//checks whether the robot has arrived at its destination
		if ((power > 0 && position < endPositionTick) || (power < 0 && position > endPositionTick)) {
        	powerDrive(-adjustedLeftPower * scalar, adjustedRightPower * scalar);
			//returns that it has not arrived at its destination
			return false;
		}else {
            powerDrive(0, 0);
			return true;
		}
	}

	public boolean gyroTurn(double targetAngleDeg) {
		double error = targetAngleDeg - getHeading();
        System.out.println("error: " + error + " heading: " + getHeading());
        double power = Math.max(Math.abs(error / 360), 0.2) * (error < 0 ? -1 : 1);
        System.out.println("Power for gyro turn: " + power);
        if(Math.abs(error) >= 2.0) {
		    powerDrive(power, power);  
        }
        return Math.abs(error) <= 2.0;
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

	public double getAverageEncoderDistance() {
		return (-frontLeftMotor.getEncoder().getPosition() + frontRightMotor.getEncoder().getPosition()) / 2.0;
	}

	public void resetEncoders() {
        frontLeftMotor.getEncoder().setPosition(0);
        frontRightMotor.getEncoder().setPosition(0);
    }

    public void resetGyro() {
        gyro.reset();
        gyro.zeroYaw();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftRate = -frontLeftMotor.getEncoder().getVelocity() * Constants.DRIVE_METERS_PER_ROTATION / 60.0;
        double rightRate = frontRightMotor.getEncoder().getVelocity() * Constants.DRIVE_METERS_PER_ROTATION / 60.0;
        return new DifferentialDriveWheelSpeeds(leftRate, rightRate);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
		accessLeftVolts = leftVolts;
		accessRightVolts = rightVolts;
        frontLeftMotor.setVoltage(leftVolts);
        backLeftMotor.setVoltage(leftVolts);
        frontRightMotor.setVoltage(-rightVolts);
        backRightMotor.setVoltage(-rightVolts);
        differentialDrive.feed();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
        // gyro defines clockwise as positive, when standard convention describes
        // clockwise as negative
        return -gyro.getRate();
    }

	@Override
	public void periodic() {

		// This method will be called once per scheduler run
		odometry.update(gyro.getRotation2d(), frontLeftMotor.getEncoder().getPosition() * Constants.GEAR_RATIO * Units.inchesToMeters(Constants.DRIVE_WHEEL_CIRCUMFERENCE_INCHES),
                frontRightMotor.getEncoder().getPosition() * Constants.GEAR_RATIO * Units.inchesToMeters(Constants.DRIVE_WHEEL_CIRCUMFERENCE_INCHES));
	}
}
