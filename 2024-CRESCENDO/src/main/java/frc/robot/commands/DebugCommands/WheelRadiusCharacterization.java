package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class WheelRadiusCharacterization extends Command {
  CommandSwerveDrivetrain drivetrain;


  public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
