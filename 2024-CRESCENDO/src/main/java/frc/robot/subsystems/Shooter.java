package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase implements AutoCloseable {

  private TalonFX leftMotor;
  private TalonFX rightMotor;

  public Shooter() {
    leftMotor = new TalonFX(Constants.ShooterConstants.leftMotorID);
    rightMotor = new TalonFX(Constants.ShooterConstants.rightMotorID);

    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftMotor.clearStickyFaults();
    rightMotor.clearStickyFaults();

    rightMotor.setInverted(Constants.ShooterConstants.shooterMotorInvert);
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
  }

  public void setMotor(double speed) {
    rightMotor.set(speed);
  }

  public void stopMotor() {
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void close() throws Exception {
    leftMotor.close();
    rightMotor.close();
  }

  public TalonFXSimState getSimStateLeft() {
    return leftMotor.getSimState();
  }

  public TalonFXSimState getSimStateRight() {
    return rightMotor.getSimState();
  }

  public StatusSignal<Double> getMotorDutyCycle() {
    return rightMotor.getDutyCycle();
  }

  public void setControl(DutyCycleOut dutyCycleOut) {
    rightMotor.setControl(dutyCycleOut);
  }
}
