// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Transport;

public class AutonFactory extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Intake intake;
  private Transport transport;
  private ShooterPivot pivot;
  private Shooter shooter;
  private Elevator elevator;

  /** Creates a new AutonFactory. */
  public AutonFactory(CommandSwerveDrivetrain drivetrain, Intake intake, Transport transport, ShooterPivot pivot, Shooter shooter, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake, transport, pivot, shooter, elevator);
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.transport = transport;
    this.pivot = pivot;
    this.shooter = shooter;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the "encoded" path string from smartDashboard
    String pathAsString = SmartDashboard.getString("Auton Path", "");
    int len = pathAsString.length();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
