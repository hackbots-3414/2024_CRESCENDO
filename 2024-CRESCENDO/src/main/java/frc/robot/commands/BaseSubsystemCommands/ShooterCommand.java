package frc.robot.commands.BaseSubsystemCommands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterCommand extends Command {
  Shooter shooter;
  Transport transport;
  double velocity;
  boolean alreadyRan;
  boolean sawNote;
  boolean previousValue; // this is the previous value for "do we see a note with the IR sensor?"

  public ShooterCommand(Shooter shooter, Transport transport, double velocity) {
    // addRequirements(shooter);
    this.shooter = shooter;
    this.transport = transport;
    this.velocity = velocity;
  }

  @Override
  public void initialize() {
        shooter.setVelocity(velocity);
        alreadyRan = false;
        sawNote = false;
        previousValue = false;
  }

  @Override
  public void execute() {
    if (previousValue == false && transport.getFlyWheelIR() == true) sawNote = true; // if our value has went from false to true, then we increment the counter

    if (shooter.shooterAtSpeed() && !alreadyRan) { // Note in position and shooter at Speed
      transport.setFast();
      alreadyRan = true;
    }

    previousValue = transport.getFlyWheelIR();

  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    transport.stopMotor();
    transport.setNoteOnBoard(false);
  }

  @Override
  public boolean isFinished() {
      return !transport.getFlyWheelIR() && sawNote;
  }
}
