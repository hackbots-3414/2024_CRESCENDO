package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  private TalonFX hoodMotor;

  public Hood() {
    hoodMotor = new TalonFX(Constants.HoodConstants.hoodMotorID);
    hoodMotor.getConfigurator().apply(new TalonFXConfiguration());
    hoodMotor.clearStickyFaults();
  }

  public void startMotor() {
    hoodMotor.set(Constants.HoodConstants.hoodMotorSpeed);
  }

  public void stopMotor() {
    hoodMotor.set(0);
  }

  public void setIdle() {
    hoodMotor.set(Constants.HoodConstants.idleSpeed);
  }

  public double getMotorSpeed() {
    return hoodMotor.getRotorVelocity().getValue();
  }

  @Override
  public void periodic() {
  }
}
