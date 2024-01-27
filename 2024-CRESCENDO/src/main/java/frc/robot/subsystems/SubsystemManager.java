package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemManager extends SubsystemBase implements AutoCloseable {
    
  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  List<SubsystemBase> subsystems = new ArrayList<>();

  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  ShooterPivot shooterPivot = new ShooterPivot();
  Transport transport = new Transport();

  double totalCurrent = 0;

  double elevatorCurrent = 0;
  double intakeCurrent = 0;
  double shooterCurrent = 0;
  double shooterPivotCurrent = 0;
  double transportCurrent = 0;

  public SubsystemManager() {}

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
  }

  @Override
  public void close() throws Exception {}  
}