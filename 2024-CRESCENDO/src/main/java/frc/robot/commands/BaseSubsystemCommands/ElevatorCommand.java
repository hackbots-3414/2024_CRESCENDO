package frc.robot.commands.BaseSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ElevatorCommand extends Command {
    public enum ElevatorPresets {STOW, AMP, SUBWOOFER;}

    Elevator elevator;
    ShooterPivot shooterPivot;
    ElevatorPresets selector;

    double pivotWait = 0;

    public ElevatorCommand(Elevator elevator, ShooterPivot shooterPivot, ElevatorPresets selector) {
        addRequirements(elevator, shooterPivot);
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.selector = selector;
    }

    @Override
    public void execute() {
        switch (selector) {
            case STOW:
                elevator.stow();
                shooterPivot.stow();
                break;
            case AMP:
                elevator.setElevatorPosition(PositionConstants.AmpPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.AmpPresets.shooter);
                break;
            case SUBWOOFER:
                elevator.setElevatorPosition(PositionConstants.SubwooferPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.SubwooferPresets.shooter);
                pivotWait++;
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetpoint() && shooterPivot.isAtSetpoint() && (pivotWait > 15 || selector != ElevatorPresets.SUBWOOFER);
    }
}