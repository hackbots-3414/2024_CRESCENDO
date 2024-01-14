package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ShooterPivot extends ProfiledPIDSubsystem {

  private final TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotorID);
  private final CANcoder cancoder = new CANcoder(Constants.PivotConstants.EncoderID);

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

    cancoder.clearStickyFaults();

    // FIX ME for CANCODER use
    // m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);

    // Start arm at rest in neutral position
    setGoal(Constants.PivotConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = m_Feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    pivotMotor.setControl(new VoltageOut(output + feedforward));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    //Check get pos
    return cancoder.getPosition().getValueAsDouble() + Constants.PivotConstants.kArmOffsetRads;
  }
}
