package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.Telemetry;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TransportCommand;
import frc.robot.generated.TunerConstants;

public class SubsystemManager extends SubsystemBase {
    
  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  List<SubsystemBase> subsystems = new ArrayList<>();

  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  ShooterPivot shooterPivot = new ShooterPivot();
  Transport transport = new Transport();
  NoteFinder noteFinder = new NoteFinder();

  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.1).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveDriveBrake brakeRequest = new SwerveDriveBrake();
  PointWheelsAt pointRequest = new PointWheelsAt();
  Telemetry logger = new Telemetry();

  double elevatorCurrent = 0;
  double intakeCurrent = 0;
  double shooterCurrent = 0;
  double shooterPivotCurrent = 0;
  double transportCurrent = 0;

  double inputCurrent = 18; // 18 AMP HOURS
  double runTimeHours = 0.05; // 3 MINUTES
  double coprocessorsAmpRating = 3*2 * runTimeHours; // 3 AMP HOURS for runTimeHours per coprocessor
  double availableCurrent = inputCurrent - coprocessorsAmpRating;

  public SubsystemManager() {}

  public Intake getIntake() {return intake;}
  public Shooter getShooter() {return shooter;}
  public ShooterPivot getShooterPivot() {return shooterPivot;}
  public Transport getTransport() {return transport;}
  public Elevator getElevator() {return elevator;}
  public NoteFinder getNoteFinder() {return noteFinder;}

  @Override
  public void periodic() {
    intake.periodic();
    shooter.periodic();
    shooterPivot.periodic();
    transport.periodic();
    elevator.periodic();

    elevatorCurrent = pdp.getCurrent(ElevatorConstants.elevatorMotorPDPID) + pdp.getCurrent(ElevatorConstants.elevatorFollowerMotorPDPID);
    intakeCurrent = pdp.getCurrent(IntakeConstants.intakeMotorPDPID);
    shooterPivotCurrent = pdp.getCurrent(PivotConstants.pivotMotorPDPID);
    shooterCurrent = pdp.getCurrent(ShooterConstants.leftMotorID) + pdp.getCurrent(ShooterConstants.rightMotorID);
    transportCurrent = pdp.getCurrent(TransportConstants.transportMotorPDPID);

    dampenDrivetrain();
  }

  private void dampenDrivetrain() {
    double supplyLimitDrivetrain = ((availableCurrent / runTimeHours - (elevatorCurrent + intakeCurrent + shooterPivotCurrent + shooterCurrent + transportCurrent)))/4.0; // (Ah Available - Ah Being Used) / Ah to Amps conversion / 4 motors to distribute over
    supplyLimitDrivetrain = supplyLimitDrivetrain > 40 ? 39.5 : supplyLimitDrivetrain;
    SmartDashboard.putNumber("CURRENT LIMIT FOR DRIVETRAIN", supplyLimitDrivetrain);
    drivetrain.setCurrentLimit(supplyLimitDrivetrain);
  }

  public void configureDriveDefaults(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> driveRequest.withVelocityX(-x.get() * Constants.SwerveConstants.maxDriveVelocity).withVelocityY(-y.get() * Constants.SwerveConstants.maxDriveVelocity).withRotationalRate(-turn.get() * Constants.SwerveConstants.maxAngleVelocity)));
  }

  public Command makeBrakeCommand() {return drivetrain.applyRequest(() -> brakeRequest);}
  public Command makePointCommand(double x, double y) {return drivetrain.applyRequest(() -> pointRequest.withModuleDirection(new Rotation2d(-x, -y)));}
  public Command makeResetCommand() {return drivetrain.runOnce(() -> drivetrain.seedFieldRelative());}
  public void resetAtPose2d(Pose2d pose) {drivetrain.seedFieldRelative(pose);}

  public void telemeterize() {drivetrain.registerTelemetry(logger::telemeterize);}

  public Command makeElevatorCommand(ElevatorPresets preset) {return new ElevatorCommand(elevator, shooterPivot, preset);}
  public Command makeShootCommand(double speed) {return new ShooterCommand(shooter, Constants.ShooterConstants.shootSpeed);}
  public Command makeIntakeCommand() {return new IntakeCommand(transport, intake, Constants.IntakeConstants.intakeSpeed, Constants.TransportConstants.transportSpeed);}
  public Command makeEjectCommand() {return new IntakeCommand(transport, intake, Constants.IntakeConstants.ejectSpeed, Constants.TransportConstants.transportEjectSpeed);}
  public Command makeTransportCommand(boolean forward) {return new TransportCommand(transport, forward);}
}