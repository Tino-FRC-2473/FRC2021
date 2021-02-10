package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPrototypeSubsystem extends SubsystemBase {

  private CANSparkMax shooter;
  private double targetPower;

  public ShooterPrototypeSubsystem() {
    shooter = new CANSparkMax(ShooterConstants.SPARK_SHOOTER, MotorType.kBrushless);
    targetPower = 0.3;
    shooter.set(0); // on init, set power to 0
  }

  public void runShooterPower(double power) {
    shooter.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getPower() {
    return shooter.get();
  }

  public double getTargetPower() {
    return targetPower;
  }

  public void setTargetPower(double targetPower) {
    this.targetPower = targetPower;
  }

}