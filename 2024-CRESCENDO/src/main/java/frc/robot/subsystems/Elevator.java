package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.math.Conversions;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends ProfiledPIDSubsystem implements AutoCloseable {

  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
  private TalonFX elevatorFollower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
  private CANcoder elevatorCanCoder = new CANcoder(ElevatorConstants.elevatorCANCoderMotorID);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.elevatorkS, ElevatorConstants.elevatorkG, ElevatorConstants.elevatorkV, ElevatorConstants.elevatorkA);

  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
  private final static ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, m_constraints);

  private double elevatorPosition;
  private double elevatorCanCoderVelocity;
  private double elevatorCanCoderPosition;

  private boolean isRunning = false;

  public Elevator() {
    super(controller, 0);

    configElevatorEncoder();
    Timer.delay(0.1);
    configElevatorMotors();

    m_controller.reset(getMeasurement(), getCanCoderVelo());
    m_controller.enableContinuousInput(ElevatorConstants.elevatorLowerLimit, ElevatorConstants.elevatorUpperLimit);

    setGoal(getMeasurement());
  }

  private void configElevatorMotors() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);
    elevatorFollower.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.metersToFalcon(ElevatorConstants.elevatorUpperLimit, ElevatorConstants.circumference, ElevatorConstants.gearRatio); // DO THE MATH
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.metersToFalcon(ElevatorConstants.elevatorUpperLimit, ElevatorConstants.circumference, ElevatorConstants.gearRatio); // DO THE MATH
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.elevatorCurrentLimit;
    configuration.CurrentLimits.SupplyCurrentThreshold = 0;
    configuration.CurrentLimits.SupplyTimeThreshold = 0;

    elevatorMotor.setSafetyEnabled(true);
    elevatorMotor.setPosition(Conversions.metersToFalcon(getCanCoder(), ElevatorConstants.circumference, ElevatorConstants.gearRatio), 100);  

    elevatorMotor.getConfigurator().apply(configuration, 0.2);
    elevatorFollower.setControl(new Follower(ElevatorConstants.elevatorMotorID, false));
  }

  private void configElevatorEncoder() {
    elevatorCanCoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration configuration = new CANcoderConfiguration();

    configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    configuration.MagnetSensor.MagnetOffset = 0; //*********** FIGURE THIS OUT */
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    elevatorCanCoder.getConfigurator().apply(configuration);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //calculate feedforward from setpoint
    double feedforwardoutput = feedforward.calculate(setpoint.velocity);
    elevatorMotor.setVoltage(output + feedforwardoutput);
  }

  @Override
  public double getMeasurement() {return Math.toRadians(getCanCoder());}

  public void set(double speed) {elevatorMotor.set(speed);} // -1 to 1
  public void stop() {elevatorMotor.set(0.0);}

  public double getPosition() {return elevatorPosition;}
  public double getCanCoder() {return elevatorCanCoderPosition;}
  public double getCanCoderVelo() {return Math.toRadians(elevatorCanCoderVelocity);}

  public void setNeutralMode(NeutralModeValue value) {elevatorMotor.setNeutralMode(value);}

  public void setCurrentLimit(double limit) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(limit);
    elevatorMotor.getConfigurator().apply(configs, 0.01);
    elevatorFollower.getConfigurator().apply(configs, 0.01);
  }

  public void setRunning(boolean isRunning) {this.isRunning = isRunning;}
  public boolean getRunning() {return this.isRunning;}

  @Override
  public void periodic() {
    super.periodic();
    elevatorMotor.feed();

    elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    elevatorCanCoderVelocity = elevatorCanCoder.getVelocity().getValueAsDouble();
    elevatorCanCoderPosition = elevatorCanCoder.getAbsolutePosition().getValueAsDouble();
    // System.out.println(elevatorPosition);
  }

  @Override
  public void close() throws Exception{
    elevatorMotor.close();
    elevatorFollower.close();
    elevatorCanCoder.close();
  }

  public TalonFXSimState getSimState() {return elevatorMotor.getSimState();}
}