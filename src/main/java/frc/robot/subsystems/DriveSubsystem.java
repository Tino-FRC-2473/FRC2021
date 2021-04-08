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
        double power = Math.max(Math.abs(error / 360), 0.3) * (error < 0 ? -1 : 1);
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
		double leftPower = Robot.robotContainer.getLeftY();
		double rightPower = Robot.robotContainer.getRightY();
		if(Math.abs(leftPower - rightPower) < 0.1) {
			leftPower = (leftPower + rightPower) / 2;
			rightPower = leftPower;
		}
		differentialDrive.tankDrive(leftPower * 0.4, rightPower * 0.4, false) ;

	}

	public void arcadeDrive() {
		double rightX = -Robot.robotContainer.getRightX();
		double leftY = Robot.robotContainer.getLeftY();
		System.out.println("RightX : " + rightX + "   leftY: " + leftY);
		//differentialDrive.arcadeDrive(Robot.robotContainer.getLeftY() * 0.6, -Robot.robotContainer.getRightX() * 0.4, true); // for non carpet
		differentialDrive.arcadeDrive(leftY * 0.6, rightX * 1.5, true); // for carpet
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

    public void resetGyro() {
        gyro.reset();
        gyro.zeroYaw();
    }

	@Override
	public void periodic() {

		// This method will be called once per scheduler run
	}
}
