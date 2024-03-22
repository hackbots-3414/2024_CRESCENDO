package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorMotionMagicConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSlot0ConfigConstants;

public class Elevator extends SubsystemBase implements AutoCloseable {

  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
  private DigitalInput forwardLimiter = new DigitalInput(Constants.ElevatorConstants.forwardLimitChannelID);
  private DigitalInput reverseLimiter = new DigitalInput(Constants.ElevatorConstants.reverseLimitChannelID);

  private double elevatorPosition;
  private boolean forwardLimit;
  private boolean reverseLimit;

  private double setpoint;

  public Elevator() {
    configElevatorMotors();
  }

  private void configElevatorMotors() {
    elevatorMotor.clearStickyFaults();

    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    TalonFXConfiguration configuration = new TalonFXConfiguration()

        .withSlot0(new Slot0Configs()
            .withKP(ElevatorSlot0ConfigConstants.kP)
            .withKI(ElevatorSlot0ConfigConstants.kI)
            .withKD(ElevatorSlot0ConfigConstants.kD)
            .withKS(ElevatorSlot0ConfigConstants.kS)
            .withKV(ElevatorSlot0ConfigConstants.kV)
            .withKA(ElevatorSlot0ConfigConstants.kA)
            .withKG(ElevatorSlot0ConfigConstants.kG)
            .withGravityType(GravityTypeValue.Elevator_Static))

        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ElevatorMotionMagicConstants.cruiseVelocity)
            .withMotionMagicAcceleration(ElevatorMotionMagicConstants.acceleration)
            .withMotionMagicJerk(ElevatorMotionMagicConstants.jerk))
            
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(ElevatorConstants.invertMotor))
            
        .withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(ElevatorConstants.rotorToSensorRatio)
            .withSensorToMechanismRatio(ElevatorConstants.sensorToMechanismRatio))
            
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimits.elevatorSupplyLimit))
            
        .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
            .withForwardLimitEnable(true)
            .withReverseLimitEnable(true)
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0.0))
            
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(ElevatorConstants.elevatorForwardSoftLimit)
            .withForwardSoftLimitEnable(true));

    elevatorMotor.getConfigurator().apply(configuration, 0.2);
  }

  public void setElevatorPosition(double position) { // position is in number of rotations as per documentation.
    MotionMagicVoltage config = new MotionMagicVoltage(position);
    if (position < this.elevatorPosition) {
      config.withLimitReverseMotion(getReverseLimit());
    } else if(position >= this.elevatorPosition){
      config.withLimitForwardMotion(getForwardLimit());
    }
    this.setpoint = position;
    elevatorMotor.setControl(config);
  }

  public void stow() {
    setElevatorPosition(PositionConstants.StowPresets.elevator);
  }

  public void set(double speed) {
    if ((speed > 0 && !getForwardLimit()) || (speed < 0 && getReverseLimit())) {
      elevatorMotor.setControl(new DutyCycleOut(speed)
        .withLimitForwardMotion(getForwardLimit())
        .withLimitReverseMotion(getReverseLimit()));
    } else {
      elevatorMotor.stopMotor();
    }
  }

  public void setResetElevatorSpeed() {
    elevatorMotor.setControl(new DutyCycleOut(ElevatorConstants.resetElevatorSpeed)
        .withLimitForwardMotion(getForwardLimit())
        .withLimitReverseMotion(getReverseLimit()));
  }

  public void setElevatorUpSpeed() {
    set(ElevatorConstants.elevatorManualUpSpeed);
  }

  public void setElevatorDownSpeed() {
    set(ElevatorConstants.elevatorManualDownSpeed);
  }

  public boolean isAtSetpoint() {
    return Math.abs(getPosition() - setpoint) < ElevatorConstants.elevatorTolerance;
  }

  public void stop() {
    set(0.0);
  }

  public boolean getForwardLimit() {
    return forwardLimit;
  }

  public boolean getReverseLimit() {
    return reverseLimit;
  }

  public double getPosition() {
    return elevatorPosition;
  }

  public void setNeutralMode(NeutralModeValue value) {
    elevatorMotor.setNeutralMode(value);
  }

  @Override
  public void periodic() {
    elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();

    reverseLimit = !reverseLimiter.get();
    forwardLimit = !forwardLimiter.get();

    if (DebugConstants.debugMode) {
      SmartDashboard.putBoolean("Forward Limit switch", forwardLimit);
      SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimit);
      SmartDashboard.putNumber("Elevator Position", elevatorPosition);
      SmartDashboard.putBoolean("ELEVATOR SETPOINT", isAtSetpoint());
    }
  }

  @Override
  public void close() throws Exception {
    elevatorMotor.close();
    forwardLimiter.close();
    reverseLimiter.close();
  }

  public TalonFXSimState getSimState() {
    return elevatorMotor.getSimState();
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Angle> m_distance = mutable(Radians.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            elevatorMotor.setControl(new VoltageOut(volts.in(Volts)).withLimitForwardMotion(forwardLimit).withLimitReverseMotion(reverseLimit));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("Elevator Motors")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        elevatorMotor.get() * RobotController.getBatteryVoltage(), Volts))
                // .angularPosition(m_distance.mut_replace(getMeasurement(), Radians))
                // .angularVelocity(m_velocity.mut_replace(getCanCoderVelo(),
                // RadiansPerSecond));
                .angularPosition(m_distance.mut_replace(Units.rotationsToRadians(elevatorPosition), Radians))
                .angularVelocity(m_velocity.mut_replace(Units.rotationsPerMinuteToRadiansPerSecond(elevatorMotor.getVelocity().getValueAsDouble()),
                    RadiansPerSecond));
          },

          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("Shooter")
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}