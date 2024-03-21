// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ManualShootCommand extends Command {
  private Shooter shooter;
  private Transport transport;

  /** Creates a new ManualShoot. */
  public ManualShootCommand(Shooter shooter, Transport transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.transport = transport;
    addRequirements(shooter);
    addRequirements(transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.setVelocity(Constants.ShooterConstants.spitOutSpeed);
    shooter.setMotor(0.3);
    transport.setFast();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
