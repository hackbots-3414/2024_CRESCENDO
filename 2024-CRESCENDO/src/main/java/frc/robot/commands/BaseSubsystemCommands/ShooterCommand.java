package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
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
  }

  @Override
  public void initialize() {
    alreadyRan = false;
    previousValue = false;
  }

  @Override
  public void execute() {
    // if (previousValue == false && transport.getFlyWheelIR() == true) sawNote = true; // if our value has went from false to true, then we increment the counter

    if(transport.getFlyWheelIR() && transport.getTransportIR()) {
      shooter.setMaxSpeed();
      if(shooter.shooterAtSpeed()){
        transport.setSlow();
        alreadyRan = true;
      }
    }

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
