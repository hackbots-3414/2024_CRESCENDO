package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorMotionMagicConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSlot0ConfigConstants;
import frc.robot.Constants.PivotConstants.PivotMotionMagicConstants;
import frc.robot.Constants.PivotConstants.PivotSlot0ConfigConstants;

public class ShooterPivot extends SubsystemBase implements AutoCloseable{

  private final TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotorID);
  private final CANcoder cancoder = new CANcoder(Constants.PivotConstants.EncoderID);

  private double cancoderPosition;
  private double cancoderVelocity;
  
  private boolean isRunning = false;

    private Slot0Configs slot0Config = new Slot0Configs()
    .withKP(PivotSlot0ConfigConstants.kP)
    .withKI(PivotSlot0ConfigConstants.kI)
    .withKD(PivotSlot0ConfigConstants.kD)
    .withKS(PivotSlot0ConfigConstants.kS)
    .withKV(PivotSlot0ConfigConstants.kV)
    .withKA(PivotSlot0ConfigConstants.kA);

  private MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(PivotMotionMagicConstants.cruiseVelocity)
    .withMotionMagicAcceleration(PivotMotionMagicConstants.acceleration)
    .withMotionMagicJerk(PivotMotionMagicConstants.jerk);
  
  private MotionMagicVoltage m_request = new MotionMagicVoltage(0.0); // FIXME inital pos might be current pos insted of 0

  public ShooterPivot() {
    configMotor();
    configEncoder();
    Timer.delay(0.5);
  }

  public void setPivotPosition(double position) { // position is in number of rotations as per documentation.
    pivotMotor.setControl(m_request.withPosition(position));
  }

  public double getCancoderVelo() {return cancoderVelocity;}
  public double getCancoderPos() {return cancoderPosition;}

  @Override
  public void periodic() {
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

    TalonFXConfiguration configuration = new TalonFXConfiguration()
    .withSlot0(slot0Config)
    .withMotionMagic(motionMagicConfig);

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

    pivotMotor.getConfigurator().apply(configuration);
  }

    public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);
    pivotMotor.getConfigurator().apply(configs, 0.01);
  }

  public void setRunning(boolean isRunning) {this.isRunning = isRunning;}
  public boolean getRunning() {return this.isRunning;}

  @Override
  public void close() throws Exception {
    pivotMotor.close();
    cancoder.close();
  }

}