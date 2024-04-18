// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class SubwooferSetupCommand extends Command {
  private Shooter shooter;
  private ShooterPivot pivot;
  private boolean setup;
  /** Creates a new SubwooferSetupCommand. */
  public SubwooferSetupCommand(ShooterPivot pivot, Shooter shooter) {
    this.shooter = shooter;
    this.pivot = pivot;
    addRequirements(pivot, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setup = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!setup) {
      shooter.setVelocity(Constants.ShooterConstants.subwooferVelocity);
      pivot.setSubwoofer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
