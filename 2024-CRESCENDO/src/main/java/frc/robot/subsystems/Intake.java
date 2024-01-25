package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements AutoCloseable {

  TalonFX intakeMotor;

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
    intakeMotor.set(speed);
  }

  public void stopMotor() {
    intakeMotor.set(0);
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

  @Override
  public void close() throws Exception {
    intakeMotor.close();
  }  
}