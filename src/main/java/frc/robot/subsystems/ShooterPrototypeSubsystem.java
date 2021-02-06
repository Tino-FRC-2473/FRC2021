package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPrototypeSubsystem extends SubsystemBase {

  private CANSparkMax shooter;

  public ShooterPrototypeSubsystem() {
    shooter = new CANSparkMax(ShooterConstants.SPARK_SHOOTER, MotorType.kBrushless);
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

  public double getSpeed() {
    return shooter.get();
  }

}