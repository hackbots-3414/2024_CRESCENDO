package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements AutoCloseable {

  TalonFX intakeMotor;

  DigitalInput m_forwardLimit = new DigitalInput(0);
  DutyCycleOut m_DutyCycleOut = new DutyCycleOut(0.0);
  
  private boolean isRunning = false;

  public Intake() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);
    intakeMotor.clearStickyFaults();
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInvert);
  }

  @Override
  public void periodic() {
  }

  public void setMotor(double speed) {
    // intakeMotor.set(speed);
    intakeMotor.setControl(m_DutyCycleOut.withOutput(speed).withLimitForwardMotion(!m_forwardLimit.get()));
  }

  public void stopMotor() {
    intakeMotor.setControl(m_DutyCycleOut.withOutput(0.0));
  }

  public boolean getForwardLimit() {
    return !m_forwardLimit.get();
  }

  public double getSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public TalonFXSimState getSimState() {
    return intakeMotor.getSimState();
  }

  public StatusSignal<Double> getMotorDutyCycle() {
    return intakeMotor.getDutyCycle();
  }

  public void setControl(ControlRequest request) {
    intakeMotor.setControl(request);
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);
    intakeMotor.getConfigurator().apply(configs, 0.01);
  }

  public void setRunning(boolean isRunning) {this.isRunning = isRunning;}
  public boolean getRunning() {return this.isRunning;}

  @Override
  public void close() {
    intakeMotor.close();
    m_forwardLimit.close();
  }
}