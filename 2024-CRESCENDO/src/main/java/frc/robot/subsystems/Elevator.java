package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Elevator extends PIDSubsystem {

  private TalonFX elevator;
  private TalonFX elevatorFollower;

  public Elevator() {
    super(new PIDController(0, 0, 0));
    elevator.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollower.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollower.setControl(new Follower(Constants.ElevatorConstants.elevatorMotorID, true));
  }

  @Override
  public void periodic() {
  }

@Override
protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
}

@Override
protected double getMeasurement() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
}
}
