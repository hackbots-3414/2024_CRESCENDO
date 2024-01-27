package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ShooterPivot extends ProfiledPIDSubsystem {

  private final TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotorID);
  private final CANcoder cancoder = new CANcoder(Constants.PivotConstants.EncoderID);

  private double cancoderPosition;
  private double cancoderVelocity;

  private final ArmFeedforward m_Feedforward = new ArmFeedforward(
      Constants.PivotConstants.kSVolts,
      Constants.PivotConstants.kGVolts,
      Constants.PivotConstants.kVVoltSecondPerRad,
      Constants.PivotConstants.kAVoltSecondSquaredPerRad);

  public ShooterPivot() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            Constants.PivotConstants.kP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                Constants.PivotConstants.kMaxVelocityRadPerSecond,
                Constants.PivotConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    configEncoder();
    Timer.delay(1);

    m_controller.reset(getMeasurement(), getCancoderVelo());
  }

  public double getCancoderVelo() {return cancoderVelocity;}

  public double getCancoderPos() {return cancoderPosition;}

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_Feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    pivotMotor.setControl(new VoltageOut(output + feedforward));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    //Check get pos
    return cancoder.getPosition().getValueAsDouble() + Constants.PivotConstants.kArmOffset;
  }

  @Override
  public void periodic() {
    super.periodic();
    pivotMotor.feed();

    cancoderPosition = cancoder.getAbsolutePosition().getValueAsDouble();
    cancoderVelocity = cancoder.getVelocity().getValueAsDouble(); // Rotations/s

  }

  public void configEncoder() {
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfiguration.MagnetSensor.MagnetOffset = Constants.PivotConstants.kArmOffset;
    canCoderConfiguration.MagnetSensor.SensorDirection = Constants.PivotConstants.cancoderInvert;

    cancoder.getConfigurator().apply(canCoderConfiguration);
  }

  public void configMotor() {
    pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = Constants.PivotConstants.motorInvert;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.PivotConstants.forwardSoftLimitThreshold;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.PivotConstants.reverseSoftLimitThreshold;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = Constants.PivotConstants.motorCurrentLimit;
    configuration.CurrentLimits.SupplyCurrentThreshold = 0;
    configuration.CurrentLimits.SupplyTimeThreshold = 0;
  }

    public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);
    pivotMotor.getConfigurator().apply(configs, 0.01);
  }

}