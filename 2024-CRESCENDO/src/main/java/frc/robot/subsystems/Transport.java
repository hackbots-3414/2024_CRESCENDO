package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transport extends SubsystemBase implements AutoCloseable{

  private TalonFX transportMotor;
  private DigitalInput irSensor = new DigitalInput(Constants.TransportConstants.irSensorChannel);
  
  private boolean isRunning = false;

  public Transport() {
    transportMotor = new TalonFX(Constants.TransportConstants.transportMotorID);
    transportMotor.clearStickyFaults();
    transportMotor.getConfigurator().apply(new TalonFXConfiguration());
    transportMotor.setInverted(Constants.TransportConstants.transportMotorInvert);
  }

  public void setMotor(double speed) {transportMotor.set(speed);}
  public void stopMotor() {transportMotor.set(0);}
  
  public boolean getIR() {return irSensor.get();}

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);
    transportMotor.getConfigurator().apply(configs, 0.01);
  }

  public void setRunning(boolean isRunning) {this.isRunning = isRunning;}
  public boolean getRunning() {return this.isRunning;}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IR Sensor", getIR());
  }

  @Override
  public void close() throws Exception {
    transportMotor.close();
    irSensor.close();
  }
}
