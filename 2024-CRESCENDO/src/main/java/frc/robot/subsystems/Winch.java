package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

public class Winch extends SubsystemBase implements AutoCloseable {

  private TalonFX leftWinchMotor = new TalonFX(WinchConstants.leftWinchMotorID);
  private TalonFX rightWinchMotor = new TalonFX(WinchConstants.rightWinchMotorID);
  private double motorPosition;

  private PIDController pid = new PIDController(1, 0, 0.01);

  public Winch() {
    configMotors();
  }

  public void configMotors() {
    leftWinchMotor.clearStickyFaults();
    rightWinchMotor.clearStickyFaults();

    leftWinchMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightWinchMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(WinchConstants.sensorToMechanismRatio));

    rightWinchMotor.getConfigurator().apply(configuration, 0.2);

    rightWinchMotor.setInverted(WinchConstants.winchMotorInvert);
    
    rightWinchMotor.setNeutralMode(NeutralModeValue.Brake);
    leftWinchMotor.setNeutralMode(NeutralModeValue.Brake);

    leftWinchMotor.setControl(new Follower(rightWinchMotor.getDeviceID(), false));
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

  public void setClimbUpSpeed() {
    setMotor(WinchConstants.winchManualUpSpeed);
  }

  public void setClimbDownSpeed() {
    setMotor(WinchConstants.winchManualDownSpeed);
  }

  public void stopMotor() {
    rightWinchMotor.set(0);
  }

  public double getMotorPos() {
    return motorPosition;
  }

  @Override
  public void close() {
    leftWinchMotor.close();
    rightWinchMotor.close();
  }
}
