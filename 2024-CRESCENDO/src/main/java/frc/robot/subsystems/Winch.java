package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Winch extends SubsystemBase implements AutoCloseable {

  private TalonFX leftWinchMotor = new TalonFX(Constants.ShooterConstants.leftMotorID);
  private TalonFX rightWinchMotor = new TalonFX(Constants.ShooterConstants.rightMotorID);
  private double motorPosition;

  private PIDController pid;

  public Winch() {
    configMotors();
  }

  public void configMotors() {
    leftWinchMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightWinchMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftWinchMotor.clearStickyFaults();
    rightWinchMotor.clearStickyFaults();

    rightWinchMotor.setInverted(Constants.ShooterConstants.shooterMotorInvert);
    rightWinchMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(25));

    leftWinchMotor.setControl(new Follower(rightWinchMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    motorPosition = rightWinchMotor.getPosition().getValueAsDouble();
  }

  public void setMotorPosition(double position) {
    setMotor(pid.calculate(getMotorPos(), position));
  }

  public void setMotor(double speed) {
    rightWinchMotor.set(speed);
  }

  public void stopMotor() {
    rightWinchMotor.set(0);
  }

  public double getMotorPos() {
    return motorPosition;
  }

  public double getMotorPoseDegrees() {
    return getMotorPosRad();
  }

  public double getMotorPosRad() {
    return getMotorPos() * 2.0 * Math.PI;
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);
    leftWinchMotor.getConfigurator().apply(configs, 0.01);
    rightWinchMotor.getConfigurator().apply(configs, 0.01);
  }

  @Override
  public void close() {
    leftWinchMotor.close();
    rightWinchMotor.close();
  }
}
