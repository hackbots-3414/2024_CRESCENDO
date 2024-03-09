// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class AmpSetup extends Command {
  private Elevator elevator;
  private ShooterPivot pivot;
  private double elevatorGoal = Constants.PositionConstants.AmpPresets.elevator;
  private double pivotGoal = Constants.PositionConstants.AmpPresets.shooter;
  /** Creates a new AmpSetup. */
  public AmpSetup(Elevator elevator, ShooterPivot pivot) {
    addRequirements(elevator, pivot);
    this.elevator = elevator;
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorPosition(elevatorGoal);
    pivot.setPivotPosition(pivotGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevator.getPosition() != elevatorGoal) return false;
    if (pivot.getCancoderPos() != pivotGoal) return false;
    return true;
  }
}
