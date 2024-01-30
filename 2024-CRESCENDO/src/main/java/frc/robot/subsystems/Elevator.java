package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends ProfiledPIDSubsystem implements AutoCloseable{

  private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
  private TalonFX elevatorFollower = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
  private CANcoder elevatorCanCoder = new CANcoder(ElevatorConstants.elevatorCANCoderMotorID);

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.elevatorkS, ElevatorConstants.elevatorkG, ElevatorConstants.elevatorkV, ElevatorConstants.elevatorkA);

  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
  private final static ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, m_constraints);

  private double elevatorPosition;
  private double elevatorCanCoderVelocity;
  private double elevatorCanCoderPosition;

  
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
            elevatorMotor.setVoltage(volts.in(Volts));
            elevatorFollower.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("Elevator Motors")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        getMotorSpeed() * RobotController.getBatteryVoltage(), Volts))
                // .angularPosition(m_distance.mut_replace(getMeasurement(), Radians))
                // .angularVelocity(m_velocity.mut_replace(getCanCoderVelo(), RadiansPerSecond));
                .angularPosition(m_distance.mut_replace(getMotorPosRad(), Radians))
                .angularVelocity(m_velocity.mut_replace(getMotorVeloRad(), RadiansPerSecond));
          },

          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("Shooter")
          this));

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

  public double getMotorSpeed() {return (elevatorMotor.get() + elevatorFollower.get()) / 2.0;}

  public double getMotorPos() {return ((elevatorMotor.getPosition().getValueAsDouble() + elevatorFollower.getPosition().getValueAsDouble()) / 2.0);}
  
  public double getMotorVelo() {return ((elevatorMotor.getVelocity().getValueAsDouble() + elevatorFollower.getVelocity().getValueAsDouble()) / 2.0);}

  public double getMotorPosRad() {return Units.rotationsToRadians(getMotorPos());}

  public double getMotorVeloRad() {return Units.rotationsPerMinuteToRadiansPerSecond(getMotorVelo() * 60);}

   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    super.periodic();
    elevatorMotor.feed();

    elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    elevatorCanCoderVelocity = elevatorCanCoder.getVelocity().getValueAsDouble();
    elevatorCanCoderPosition = elevatorCanCoder.getAbsolutePosition().getValueAsDouble();
    System.out.println(elevatorPosition);
  }

  @Override
  public void close() throws Exception{
    elevatorMotor.close();
    elevatorFollower.close();
    elevatorCanCoder.close();
  }

  public TalonFXSimState getSimState() {
    return elevatorMotor.getSimState();
  }
}