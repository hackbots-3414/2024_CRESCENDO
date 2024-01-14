package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends ProfiledPIDSubsystem {

  private TalonFX elevator = new TalonFX(ElevatorConstants.elevatorMotorID);
  private TalonFX elevatorFollower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
  private CANcoder elevatorCanCoder = new CANcoder(ElevatorConstants.elevatorCANCoderMotorID);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.elevatorkS, ElevatorConstants.elevatorkG, ElevatorConstants.elevatorkV, ElevatorConstants.elevatorkA);

  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(0, 0);
  private final static ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, m_constraints);

  private double elevatorPosition;
  private double elevatorCanCoderVelocity;
  private double elevatorCanCoderPosition;

  public Elevator() {
    super(controller, 0);

    elevator.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollower.getConfigurator().apply(new TalonFXConfiguration());

    configElevatorEncoder();
    Timer.delay(0.1);
    configElevatorMotors();

    controller.reset(0, 0);
    controller.enableContinuousInput(ElevatorConstants.elevatorLowerLimit, ElevatorConstants.elevatorUpperLimit);

    setGoal(0);
  }

  private void configElevatorMotors() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.DutyCycleNeutralDeadband = 0.0;
    configuration.MotorOutput.PeakForwardDutyCycle = 1.0;
    configuration.MotorOutput.PeakReverseDutyCycle = -1.0;

    configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

    configuration.Voltage.PeakForwardVoltage = 16.0;
    configuration.Voltage.PeakReverseVoltage = -16.0;

    configuration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(outputUnitsToEncoderUnits(ElevatorConstants.maxOutputUnits)).withForwardSoftLimitEnable(true);
    configuration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(outputUnitsToEncoderUnits(ElevatorConstants.minOutputUnits)).withReverseSoftLimitEnable(true);

    configuration.MotionMagic.MotionMagicCruiseVelocity = (ElevatorConstants.maxEncoderVelocity) * 10.0 / 4096; // CANCODER TICKS - 4096, TALONFX - 2048
    configuration.MotionMagic.MotionMagicAcceleration = (ElevatorConstants.maxEncoderVelocity) * 10.0 / 4096; // CANCODER TICKS - 4096, TALONFX - 2048

    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configuration.Slot0.kP = ElevatorConstants.kP;
    configuration.Slot0.kI = ElevatorConstants.kI;
    configuration.Slot0.kD = ElevatorConstants.kD;
    configuration.Slot0.kV = ElevatorConstants.kF;

    configuration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit;
    configuration.CurrentLimits.SupplyCurrentThreshold = ElevatorConstants.supplyCurrentLimit;
    configuration.CurrentLimits.SupplyTimeThreshold = 0.25;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevator.getConfigurator().apply(configuration, 200);
    elevatorFollower.setControl(new Follower(ElevatorConstants.elevatorMotorID, false));
  }

  private void configElevatorEncoder() {
  }

  private double encoderUnitsToOutputUnits(double encoderUnits) {return encoderUnits / ElevatorConstants.encoderUnitsPerOutputUnit;}

  private double outputUnitsToEncoderUnits(double outputUnits) {return outputUnits * ElevatorConstants.encoderUnitsPerOutputUnit;}

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //calculate feedforward from setpoint
    double feedforwardoutput = feedforward.calculate(setpoint.velocity);
    elevator.setVoltage(output + feedforwardoutput);
  }

  @Override
  public double getMeasurement() {
    return Math.toRadians(getCanCoder());
  }

  public void set(double speed) {
    elevator.set(speed); // -1 to 1
  }

  public void stop() {
    elevator.set(0.0);
  }

  public double getPosition() {
    return elevatorPosition;
  }

  public double getCanCoder() {
    return elevatorCanCoderPosition;
  }

  public boolean isAtPosition() {
    return Math.abs(elevatorPosition - getPosition()) <= ElevatorConstants.outputUnitTolerance;
  }

  public double getCanCoderVelo() {
    return Math.toRadians(elevatorCanCoderVelocity);
  }

  public void setNeutralMode(NeutralModeValue value) {
    elevator.setNeutralMode(value);
  }

  @Override
  public void periodic() {
    super.periodic();
    elevator.feed();

    elevatorPosition = elevator.getPosition().getValueAsDouble();
    elevatorCanCoderVelocity = elevatorCanCoder.getVelocity().getValueAsDouble();
    elevatorCanCoderPosition = elevatorCanCoder.getAbsolutePosition().getValueAsDouble();
  }
}
