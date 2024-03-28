package frc.robot.commands.ComboCommands.AmpCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class AmpComboScheduler extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Elevator elevator;
  private Shooter shooter;
  private Transport transport;
  
  public AmpComboScheduler(CommandSwerveDrivetrain drivetrain, Elevator elevator, Shooter shooter, Transport transport) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.shooter = shooter;
    this.transport = transport;
  }
  
  @Override
  public void end(boolean interrupted) {
    Command macro = new SequentialCommandGroup(
			drivetrain.makeDriveToAmpCommand(),
			new AmpSetupCommand(elevator, shooter),
			new ScoreAmpCommand(shooter, transport, elevator).withTimeout(Constants.AmpConstants.allowedShootTime),
      new InstantCommand(() -> elevator.stow())
		).onlyWhile(RobotContainer.getInstance()::getAmpButton);
    macro.schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}