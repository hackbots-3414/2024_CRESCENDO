// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

public class Shooter extends SubsystemBase {
  
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

  public Shooter() {
    leftMotor = new TalonFX(8);
  }

  @Override
  public void periodic() {
  }

  public void startMotor() {
    leftMotor.set(0.1);
  }

  public void stopMotor() {
    leftMotor.set(0);
  }
}
