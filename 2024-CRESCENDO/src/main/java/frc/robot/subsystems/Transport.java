package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase implements AutoCloseable {

  private TalonFX transportMotor = new TalonFX(TransportConstants.transportMotorID);
  private DigitalInput irSensor = new DigitalInput(TransportConstants.irSensorChannel);
  // private AnalogInput irSensor = new AnalogInput(Constants.TransportConstants.irSensorChannel);
  private boolean irValue;

  public Transport() {
    configMotor();
  }

  private void configMotor() {
    transportMotor.clearStickyFaults();

    transportMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    transportMotor.setInverted(TransportConstants.transportMotorInvert);
    transportMotor.setNeutralMode(NeutralModeValue.Brake);
    setCurrentLimit(CurrentLimits.transportSupplyLimit);
  }

  public void setMotor(double speed) {
    transportMotor.set(speed);
  }

  public void eject() {
    setMotor(TransportConstants.transportEjectSpeed);
    Timer.delay(TransportConstants.transportEjectDelay);
  }

  public void stopMotor() {
    transportMotor.set(0);
  }

  public boolean getIR() {
   // return irSensor.getValue() == 1;
   return irValue;
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);
    transportMotor.getConfigurator().apply(configs, 0.01);
  }

  @Override
  public void periodic() {
    irValue = !irSensor.get();
    if (DebugConstants.debugMode) SmartDashboard.putBoolean("IR SENSOR", irValue);
  }

  @Override
  public void close() {
    transportMotor.close();
    irSensor.close();
  }

  public TalonFXSimState getSimState() {
    return transportMotor.getSimState();
  }

  public StatusSignal<Double> getMotorDutyCycle() {
    return transportMotor.getDutyCycle();
  }

  public void setControl(DutyCycleOut out) {
    transportMotor.setControl(out);
  }
}
