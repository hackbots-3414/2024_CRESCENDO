package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.cfg.CoercionConfigs;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class Elevator extends ProfiledPIDSubsystem {

  private TalonFX elevator = new TalonFX(ElevatorConstants.elevatorMotorID);
  private TalonFX elevatorFollower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
  private CANcoder elevatorCanCoder = new CANcoder(ElevatorConstants.elevatorCANCoderMotorID);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.elevatorkS, Constants.ElevatorConstants.elevatorkG, Constants.ElevatorConstants.elevatorkV, Constants.ElevatorConstants.elevatorkA);

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
    TalonFXConfigurator elevatorConfigs = elevator.getConfigurator();

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.ElevatorConstants.currentLimit);
    SoftwareLimitSwitchConfigs softLimitsConfigs = new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(Constants.ElevatorConstants.elevatorUpperLimit).withReverseSoftLimitThreshold(Constants.ElevatorConstants.elevatorLowerLimit).withForwardSoftLimitEnable(false).withReverseSoftLimitEnable(false);
    TalonFXConfiguration configs = new TalonFXConfiguration().withMotorOutput(motorConfigs).withCurrentLimits(currentLimitsConfigs).withSoftwareLimitSwitch(softLimitsConfigs);

    elevatorConfigs.apply(configs);

    elevatorFollower.setControl(new Follower(Constants.ElevatorConstants.elevatorMotorID, true));
  }

  private void configElevatorEncoder() {
  }

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

  public double getCanCoderVelo() {
    return Math.toRadians(elevatorCanCoderVelocity);
  }

  public void setBrakeMode() {
    elevator.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoastMode() {
    elevator.setNeutralMode(NeutralModeValue.Coast);
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
