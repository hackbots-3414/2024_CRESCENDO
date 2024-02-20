package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase implements AutoCloseable {

  private TalonFX leftMotor = new TalonFX(Constants.ShooterConstants.leftMotorID);
  private TalonFX rightMotor = new TalonFX(Constants.ShooterConstants.rightMotorID);

  private double motorVelocity;
  private double motorSpeed;
  private double motorPosition;

  public Shooter() {
    configMotors();
  }

  public void configMotors() {
    leftMotor.clearStickyFaults();
    rightMotor.clearStickyFaults();

    rightMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);
    leftMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(Constants.ShooterConstants.kP)
            .withKI(Constants.ShooterConstants.kI)
            .withKD(Constants.ShooterConstants.kD)
            .withKS(Constants.ShooterConstants.kS));

    rightMotor.getConfigurator().apply(configuration, 0.2);

    rightMotor.setInverted(Constants.ShooterConstants.shooterMotorInvert);
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    motorVelocity = rightMotor.getVelocity().getValueAsDouble();
    motorPosition = rightMotor.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("VELOCITY FOR SHOOTER", motorVelocity);
    SmartDashboard.putNumber("VELOCITY REFERENCE", rightMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putBoolean("SHOOTER AT SPEED", shooterAtSpeed());
  }

  public void setVelocity(double velocity) {
    rightMotor.setControl(new VelocityVoltage(velocity));
  }

  public void setMotor(double speed) {
    rightMotor.setControl(new DutyCycleOut(speed));
  }

  public void stopMotor() {
    setMotor(0.0);
  }

  public boolean shooterAtSpeed() {
    if (rightMotor.getClosedLoopReference().getValueAsDouble() == 0) {
      return false;
    }
    return Math.abs(motorVelocity - rightMotor.getClosedLoopReference().getValueAsDouble()) < Constants.ShooterConstants.shooterTolerance;
  }

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);
    leftMotor.getConfigurator().apply(configs, 0.01);
    rightMotor.getConfigurator().apply(configs, 0.01);
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
  }

  public TalonFXSimState getSimStateLeft() {
    return leftMotor.getSimState();
  }

  public TalonFXSimState getSimStateRight() {
    return rightMotor.getSimState();
  }

  public StatusSignal<Double> getMotorDutyCycle() {
    return rightMotor.getDutyCycle();
  }

  public void setControl(DutyCycleOut dutyCycleOut) {
    rightMotor.setControl(dutyCycleOut);
  }

  public double getMotorPosRad() {
    return motorPosition * 2.0 * Math.PI;
  }

  public double getMotorSpeed() {
    return motorSpeed;
  }

  public double getMotorVeloRad() {
    return motorVelocity * 2.0 * Math.PI;
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_distance = mutable(Radians.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            leftMotor.setVoltage(volts.in(Volts));
            rightMotor.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism being characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider the entire group to be one motor.
            log.motor("Shooter Flywheel")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        getMotorSpeed() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_distance.mut_replace(getMotorPosRad(), Radians))
                .angularVelocity(m_velocity.mut_replace(getMotorVeloRad(), RadiansPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test state in WPILog with this subsystem's name ("Shooter")
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
