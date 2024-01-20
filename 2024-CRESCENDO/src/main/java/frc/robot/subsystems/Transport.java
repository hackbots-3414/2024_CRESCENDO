package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transport extends SubsystemBase {

  private TalonFX transportMotor;
  private DigitalInput irSensor = new DigitalInput(Constants.TransportConstants.irSensorChannel);

  public Transport() {
    transportMotor = new TalonFX(Constants.TransportConstants.transportMotorID);
    transportMotor.clearStickyFaults();
    transportMotor.getConfigurator().apply(new TalonFXConfiguration());
    transportMotor.setInverted(Constants.TransportConstants.transportMotorInvert);
  }

  public void startMotorforward() {
    transportMotor.set(Constants.TransportConstants.transportSpeed);
  }

  public void startMotorReverse() {
    transportMotor.set(-Constants.TransportConstants.transportSpeed);
  }

  public void stopMotor() {
    transportMotor.set(0);
  }
  
  public boolean getIR() {
    return irSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IR Sensor", getIR());
  }
}
