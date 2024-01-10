// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  
  private TalonFX motor;

  public Intake() {
    motor = new TalonFX(Constants.IntakeConstants.motorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startMotor() {
    motor.set(0.1);
  }

  public void stopMotor() {
    motor.set(0);
  }
}
