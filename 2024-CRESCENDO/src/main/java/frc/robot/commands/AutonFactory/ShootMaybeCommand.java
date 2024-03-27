// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Transport;

public class ShootMaybeCommand extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Transport transport;
  private Pose2d targetPose;

  /** Creates a new ShootMaybe. */
  public ShootMaybeCommand(CommandSwerveDrivetrain drivetrain, Transport transport) {
    this.drivetrain = drivetrain;
    this.transport = transport;
    addRequirements(drivetrain); // the subsystem manager handles the shooting with the Shooter and the ShooterPivot instances, and we only read from Transport
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calculate the best spot to shoot from TODO make this good logic
    targetPose = new Pose2d();

    if (!transport.getNoteInPosition()) {
      return; 
    }

    Command macro = new SequentialCommandGroup(
      drivetrain.makeDriveToPoseCommand(targetPose, false, Constants.AutonFactoryConstants.speedMultiplier),
      SubsystemManager.getInstance().makeAutoAimCommand(() -> 0.0, () -> 0.0, () -> 0.0)
    );

    macro.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !transport.getNoteOnBoard();
  }
}
