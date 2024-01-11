// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;

public class Shooter extends SubsystemBase {
  
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

  private double averageSpeed;

  public Shooter() {
    leftMotor = new TalonFX(Constants.ShooterConstants.leftMotorID);
    rightMotor = new TalonFX(Constants.ShooterConstants.rightMotorID);

    rightMotor.setInverted(false);

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    averageSpeed = (rightMotor.getVelocity().getValue() + leftMotor.getVelocity().getValue())/2;
  }

  public void startMotor() {
    rightMotor.set(0.1);
  }

  public void stopMotor() {
    rightMotor.set(0);
  }

  public double getMotorSpeed() {
    return averageSpeed;
  }
}
