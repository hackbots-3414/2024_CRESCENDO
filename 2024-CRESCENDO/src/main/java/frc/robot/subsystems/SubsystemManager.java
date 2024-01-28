package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorPresets;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;

public class SubsystemManager extends SubsystemBase {
    
  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  List<SubsystemBase> subsystems = new ArrayList<>();

  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  ShooterPivot shooterPivot = new ShooterPivot();
  Transport transport = new Transport();

  CommandSwerveDrivetrain drivetrain;

  double elevatorCurrent = 0;
  double intakeCurrent = 0;
  double shooterCurrent = 0;
  double shooterPivotCurrent = 0;
  double transportCurrent = 0;

  double inputCurrent = 18; // 18 AMP HOURS
  double runTimeHours = 0.05; // 3 MINUTES

  double coprocessorsAmpRating = 3*2 * runTimeHours; // 3 AMP HOURS for runTimeHours per coprocessor

  double availableCurrent = inputCurrent - coprocessorsAmpRating;

  public SubsystemManager(CommandSwerveDrivetrain drivetrain) {this.drivetrain = drivetrain;}

  public Intake getIntake() {return intake;}
  public Shooter getShooter() {return shooter;}
  public ShooterPivot getShooterPivot() {return shooterPivot;}
  public Transport getTransport() {return transport;}
  public Elevator getElevator() {return elevator;}

  @Override
  public void periodic() {
    intake.periodic();
    shooter.periodic();
    shooterPivot.periodic();
    transport.periodic();
    elevator.periodic();

    elevatorCurrent = pdp.getCurrent(2) + pdp.getCurrent(3);
    intakeCurrent = pdp.getCurrent(15);
    shooterPivotCurrent = pdp.getCurrent(14);
    shooterCurrent = pdp.getCurrent(4) + pdp.getCurrent(5);
    transportCurrent = pdp.getCurrent(16);

    dampenDrivetrain();
  }

  private void dampenDrivetrain() {
    double supplyLimitDrivetrain = ((availableCurrent - ((elevatorCurrent + intakeCurrent + shooterPivotCurrent + shooterCurrent + transportCurrent) * runTimeHours)) / runTimeHours)/4; // (Ah Available - Ah Being Used) / Ah to Amps conversion / 4 motors to distribute over
    supplyLimitDrivetrain = supplyLimitDrivetrain > 40 ? 39.5 : supplyLimitDrivetrain;
    SmartDashboard.putNumber("CURRENT LIMIT FOR DRIVETRAIN", supplyLimitDrivetrain);
    drivetrain.setCurrentLimit(supplyLimitDrivetrain);
  }

  public SwerveRequest makeDriveCommand(double x, double y, double turn) {
    return new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.05).withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(-x * Constants.SwerveConstants.maxDriveVelocity).withVelocityY(-y * Constants.SwerveConstants.maxDriveVelocity).withRotationalRate(-turn * Constants.SwerveConstants.maxAngleVelocity);
  }

  public Command makeElevatorCommand(ElevatorPresets preset) {return new ElevatorCommand(elevator, shooterPivot, preset);}
  public Command makeShootCommand(double speed) {return new ShooterCommand(shooter, Constants.ShooterConstants.shootSpeed);}
  public Command makeIntakeCommand() {return new IntakeCommand(transport, intake, Constants.IntakeConstants.intakeSpeed, Constants.TransportConstants.transportSpeed);}
  public Command makeEjectCommand() {return new IntakeCommand(transport, intake, Constants.IntakeConstants.ejectSpeed, Constants.TransportConstants.transportEjectSpeed);}
}