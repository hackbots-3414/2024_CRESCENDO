package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PivotMotionMagicConstants;
import frc.robot.Constants.PivotConstants.PivotSlot0ConfigConstants;
import frc.robot.Constants.PivotConstants.PivotSlot1ConfigConstants;
import frc.robot.Constants.PivotConstants.PivotSlot2ConfigConstants;
import frc.robot.Constants.PositionConstants;

public class ShooterPivot extends SubsystemBase implements AutoCloseable {

  private Logger logger = LoggerFactory.getLogger(ShooterPivot.class);

  private final TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotorID);
  private final CANcoder cancoder = new CANcoder(Constants.PivotConstants.EncoderID);

  private double cancoderPosition;

  private double setpoint;

  public ShooterPivot() {
    configMotor();
    configEncoder();
  }

  public void configEncoder() {
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(PivotConstants.absoluteSensorRange)
            .withSensorDirection(PivotConstants.cancoderInvert)
            .withMagnetOffset(PivotConstants.encoderOffset));

    cancoder.getConfigurator().apply(canCoderConfiguration, 0.2);
  }

  public void configMotor() {
    pivotMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    TalonFXConfiguration configuration = new TalonFXConfiguration()

        .withSlot0(new Slot0Configs()
            .withKP(PivotSlot0ConfigConstants.kP)
            .withKI(PivotSlot0ConfigConstants.kI)
            .withKD(PivotSlot0ConfigConstants.kD)
            .withKS(PivotSlot0ConfigConstants.kS)
            .withKV(PivotSlot0ConfigConstants.kV)
            .withKA(PivotSlot0ConfigConstants.kA)
            .withKG(PivotSlot0ConfigConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine))

        .withSlot1(new Slot1Configs()
            .withKP(PivotSlot1ConfigConstants.kP)
            .withKI(PivotSlot1ConfigConstants.kI)
            .withKD(PivotSlot1ConfigConstants.kD)
            .withKS(PivotSlot1ConfigConstants.kS)
            .withKV(PivotSlot1ConfigConstants.kV)
            .withKA(PivotSlot1ConfigConstants.kA)
            .withKG(PivotSlot1ConfigConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine))
        
        .withSlot2(new Slot2Configs()
            .withKP(PivotSlot2ConfigConstants.kP)
            .withKI(PivotSlot2ConfigConstants.kI)
            .withKD(PivotSlot2ConfigConstants.kD)
            .withKS(PivotSlot2ConfigConstants.kS)
            .withKV(PivotSlot2ConfigConstants.kV)
            .withKA(PivotSlot2ConfigConstants.kA)
            .withKG(PivotSlot2ConfigConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )

        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(PivotMotionMagicConstants.cruiseVelocity)
            .withMotionMagicAcceleration(PivotMotionMagicConstants.acceleration)
            .withMotionMagicJerk(PivotMotionMagicConstants.jerk))

        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(PivotConstants.EncoderID)
            .withFeedbackRotorOffset(PivotConstants.encoderOffset)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withRotorToSensorRatio(PivotConstants.rotorToSensorRatio)
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanismRatio))
            
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(PivotConstants.motorInvert))
            
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(PivotConstants.forwardSoftLimitThreshold)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PivotConstants.reverseSoftLimitThreshold)
            .withReverseSoftLimitEnable(true))
            
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CurrentLimits.shooterPivotSupplyLimit)
            .withSupplyCurrentLimitEnable(true));

    pivotMotor.getConfigurator().apply(configuration, 0.2);
  }

  public void setPivotPosition(double position) { // position is in number of rotations as per documentation.
    this.setpoint = position;
    boolean isTooClose = position > PivotConstants.howCloseIsTooCloseSlot2;
    boolean goingToZero = position < PivotConstants.goingToZeroToleranceSlot1;
    if (isTooClose) {
      pivotMotor.setControl(new MotionMagicVoltage(position).withSlot(0));
      // logger.debug("using slot 2");
      return;
    } else if (goingToZero) {
      pivotMotor.setControl(new MotionMagicVoltage(position).withSlot(1));
      return;
    }
    pivotMotor.setControl(new MotionMagicVoltage(position).withSlot(0));
  }

  public void stow() {
    setPivotPosition(PositionConstants.StowPresets.shooter);
  }

  public void set(double speed) {
    if ((speed > 0 && getCancoderPos() < PivotConstants.forwardSoftLimitThreshold) || (speed < 0 && getCancoderPos() > PivotConstants.reverseSoftLimitThreshold)) {
      pivotMotor.setControl(new DutyCycleOut(speed));
    } else {
      pivotMotor.setControl(new DutyCycleOut(0.0));
    }
  }

  public void setPivotUpSpeed() {
    set(PivotConstants.pivotManualUpSpeed);
  }

  public void setPivotDownSpeed() {
    set(PivotConstants.pivotManualDownSpeed);
  }

  public boolean isAtSetpoint() {
    if (setpoint > PivotConstants.howCloseIsTooCloseSlot2) {
      return (Math.abs(getCancoderPos() - setpoint) < PivotConstants.pivotTolerance * 8.0);
    } 
    return (Math.abs(getCancoderPos() - setpoint) < PivotConstants.pivotTolerance || getCancoderPos() < 0 && setpoint == 0.0);
  }

  public void stop() {
    set(0.0);
  }

  public double getCancoderPos() {
    return cancoderPosition;
  }

  @Override
  public void periodic() {
    cancoderPosition = cancoder.getAbsolutePosition().getValueAsDouble();
    if (DebugConstants.debugMode) {
      SmartDashboard.putNumber("CANCODERPOS", cancoderPosition);
      SmartDashboard.putNumber("CANCODER SETPOINT", pivotMotor.getClosedLoopReference().getValueAsDouble());
      SmartDashboard.putBoolean("SHOOTER PIVOT SETPOINT", isAtSetpoint());
    }
    
      SmartDashboard.putNumber("CANCODERPOS", cancoderPosition);
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);
    pivotMotor.getConfigurator().apply(configs, 0.01);
  }

  @Override
  public void close() throws Exception {
    pivotMotor.close();
    cancoder.close();
  }

  public void setControl(DutyCycleOut dutyCycle) {
    pivotMotor.setControl(dutyCycle);
  }

  public StatusSignal<Double> getMotorDutyCycle() {
    return pivotMotor.getDutyCycle();
  }

  public TalonFXSimState getSimState() {
    return pivotMotor.getSimState();
  }

}