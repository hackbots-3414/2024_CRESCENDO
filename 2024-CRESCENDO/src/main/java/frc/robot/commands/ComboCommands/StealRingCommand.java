package frc.robot.commands.ComboCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class StealRingCommand extends Command {
    Shooter shooter;
    Elevator elevator;
    ShooterPivot shooterPivot;

    Command intakeCommand;



    public StealRingCommand(Shooter shooter, Command intakeCommand, Elevator elevator, ShooterPivot shooterPivot) {
        this.shooter = shooter;
        this.intakeCommand = intakeCommand;
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
        shooterPivot.setPivotPosition(PositionConstants.StowPresets.shooter);
    }

    @Override
    public void execute() {
        intakeCommand.initialize();

        shooter.setVelocity(ShooterConstants.spitOutSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeCommand.end(true);

        shooter.stopMotor();
    }
}
