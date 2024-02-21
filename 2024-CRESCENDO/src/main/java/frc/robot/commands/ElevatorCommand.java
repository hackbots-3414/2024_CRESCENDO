package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ElevatorCommand extends Command {
    public enum ElevatorPresets {STOW, AMP, TRAP, RESET, TEST, SUBWOOFER;}

    Elevator elevator;
    ShooterPivot shooterPivot;
    ElevatorPresets selector;

    public ElevatorCommand(Elevator elevator, ShooterPivot shooterPivot, ElevatorPresets selector) {
        addRequirements(elevator); // Don't add requirements for shooter pivot -> auto aim needs requirements
        this.elevator = elevator;
        this.shooterPivot = shooterPivot;
        this.selector = selector;
    }

    @Override
    public void execute() {
        switch (selector) {
            case STOW:
                elevator.setElevatorPosition(PositionConstants.StowPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.StowPresets.shooter);
                break;
            case AMP:
                elevator.setElevatorPosition(PositionConstants.AmpPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.AmpPresets.shooter);
                break;
            case TRAP:
                elevator.setElevatorPosition(PositionConstants.TrapPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.TrapPresets.shooter);
                break;
            case TEST:
                elevator.setElevatorPosition(PositionConstants.TestPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.TestPresets.shooter);
            case RESET:
                elevator.setElevatorPosition(PositionConstants.ResetPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.ResetPresets.shooter);
            case SUBWOOFER:
                elevator.setElevatorPosition(PositionConstants.SubwooferPresets.elevator);
                shooterPivot.setPivotPosition(PositionConstants.SubwooferPresets.shooter);
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSetpoint() && shooterPivot.isAtSetpoint();
    }
}