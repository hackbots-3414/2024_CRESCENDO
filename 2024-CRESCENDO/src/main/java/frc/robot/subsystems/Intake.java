// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);
    intakeMotor.getConfigurator().clearStickyFaults();
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopMotor() {
    intakeMotor.set(0);
  }
}
