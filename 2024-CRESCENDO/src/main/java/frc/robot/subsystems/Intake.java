package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);
    intakeMotor.clearStickyFaults();
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeMotor.setInverted(false);
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
  
}