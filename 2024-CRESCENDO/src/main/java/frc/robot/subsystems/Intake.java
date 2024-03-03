package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimits;

public class Intake extends SubsystemBase implements AutoCloseable {

	private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);

	public Intake() {
		configIntakeMotor();
	}

	private void configIntakeMotor() {
    intakeMotor.clearStickyFaults();

    intakeMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    // TalonFXConfiguration configuration = new TalonFXConfiguration()
    //   	.withCurrentLimits(new CurrentLimitsConfigs()
	// 		.withStatorCurrentLimit(CurrentLimits.intakeStatorLimit)
	// 		.withStatorCurrentLimitEnable(true));

	// intakeMotor.getConfigurator().apply(configuration, 0.2);

    intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInvert);
  }

	public void setMotor(double speed) {
		intakeMotor.set(speed);
	}

	public void stopMotor() {
		intakeMotor.setControl(new DutyCycleOut(0.0));
	}

	public TalonFXSimState getSimState() {
		return intakeMotor.getSimState();
	}

	public StatusSignal<Double> getMotorDutyCycle() {
		return intakeMotor.getDutyCycle();
	}

	public void setControl(ControlRequest request) {
		intakeMotor.setControl(request);
	}

	@Override
	public void close() {
		intakeMotor.close();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("INTAKE SPEED", intakeMotor.getVelocity().getValueAsDouble());
	}
}
