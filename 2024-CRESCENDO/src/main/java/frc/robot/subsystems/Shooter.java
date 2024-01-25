package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX leftMotor;
  private TalonFX rightMotor;

 // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
 private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
 // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
 private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
 // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
 private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftMotor.setVoltage(volts.in(Volts));
                rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("Shooter")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            getMotorSpeed() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getMotorPosRad(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getMotorVeloRad(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public Shooter() {
    leftMotor = new TalonFX(Constants.ShooterConstants.leftMotorID);
    rightMotor = new TalonFX(Constants.ShooterConstants.rightMotorID);

    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftMotor.clearStickyFaults();
    rightMotor.clearStickyFaults();

    rightMotor.setInverted(Constants.ShooterConstants.shooterMotorInvert);
    
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
  }

  public void setMotor(double speed) {
    rightMotor.set(speed);
  }

  public void stopMotor() {
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
  }

  public double getMotorPos() {
    return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2.0;
  }

  public double getMotorPosRad() {
    return getMotorPos() * 2.0 * Math.PI;
  }

  public double getMotorVelo() {
    return (leftMotor.getVelocity().getValueAsDouble() + rightMotor.getVelocity().getValueAsDouble()) / 2.0;
  }

  public double getMotorSpeed() {
    return (leftMotor.get() + rightMotor.get()) / 2.0;
  }

  public double getMotorVeloRad() {
    return getMotorVelo() * 2.0 * Math.PI;
  }
}
