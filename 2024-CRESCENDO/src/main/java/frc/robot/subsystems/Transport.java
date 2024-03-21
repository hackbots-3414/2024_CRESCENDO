package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase implements AutoCloseable {

  private TalonFX transportMotor = new TalonFX(TransportConstants.transportMotorID);
  private AnalogInput transportIrSensor = new AnalogInput(Constants.TransportConstants.transportIrChannel);
  private AnalogInput flyWheelIrSensor = new AnalogInput(Constants.TransportConstants.flyWheelIrChannel);

  private boolean transportIrValue;
  private boolean flyWheelIrValue;

  public Transport() {
    configMotor();
  }

  private void configMotor() {
    transportMotor.clearStickyFaults();

    transportMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    // TalonFXConfiguration configuration = new TalonFXConfiguration()
    // .withCurrentLimits(new CurrentLimitsConfigs()
    // .withStatorCurrentLimit(CurrentLimits.transportStatorLimit)
    // .withStatorCurrentLimitEnable(true));

    // transportMotor.getConfigurator().apply(configuration, 0.2);

    transportMotor.setInverted(TransportConstants.transportMotorInvert);
    transportMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotor(double speed) {
    transportMotor.set(speed);
  }

  public void setEject() {
    setMotor(TransportConstants.transportEjectSpeed);
  }

  public void stopMotor() {
    transportMotor.set(0);
  }

  public void setFast() {
		setMotor(TransportConstants.fastTransportSpeed);
	}

  public void setMedium() {
    transportMotor.setControl(new VoltageOut(TransportConstants.mediumTransportVolts));
	}

	public void setSlow() {
    transportMotor.setControl(new VoltageOut(TransportConstants.slowTransportVolts));
	}

  public boolean getTransportIR() {
    return transportIrValue;
  }

  public boolean getFlyWheelIR() {
    return flyWheelIrValue;
  }

  public boolean getNoteInPosition() {
    return getTransportIR() && getFlyWheelIR();
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);
    transportMotor.getConfigurator().apply(configs, 0.01);
  }

  @Override
  public void periodic() {
    transportIrValue = transportIrSensor.getVoltage() > Constants.irSensorThreshold;
    double flywheelVoltage = flyWheelIrSensor.getVoltage();
    flyWheelIrValue = flywheelVoltage > Constants.irSensorThreshold;
    SmartDashboard.putNumber("FLYWHEEL VOLTAGE", flywheelVoltage);

    // if (DebugConstants.debugMode) {
      SmartDashboard.putBoolean("TRANSPORT IR SENSOR", transportIrValue);
      SmartDashboard.putBoolean("FLYWHEEL IR SENSOR", flyWheelIrValue);
    // }
    SmartDashboard.putNumber("INTAKE SPEED", transportMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void close() {
    transportMotor.close();
    transportIrSensor.close();
    flyWheelIrSensor.close();
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
