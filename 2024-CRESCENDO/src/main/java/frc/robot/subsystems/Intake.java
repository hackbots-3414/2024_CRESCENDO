package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements AutoCloseable {

	private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID);
	private AnalogInput intakeIrSensor = new AnalogInput(Constants.IntakeConstants.intakeIrChannel);
	private boolean intakeIrValue;

	public Intake() {
		configIntakeMotor();
	}

	private void configIntakeMotor() {
		intakeMotor.clearStickyFaults();

		intakeMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

		// TalonFXConfiguration configuration = new TalonFXConfiguration()
		// .withCurrentLimits(new CurrentLimitsConfigs()
		// .withStatorCurrentLimit(CurrentLimits.intakeStatorLimit)
		// .withStatorCurrentLimitEnable(true));

		// intakeMotor.getConfigurator().apply(configuration, 0.2);

		intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInvert);
	}

	public void setMotor(double speed) {
		intakeMotor.set(speed);
	}

	public void stopMotor() {
		intakeMotor.setControl(new DutyCycleOut(0.0));
	}

	public void setFast() {
		setMotor(IntakeConstants.fastIntakeSpeed);
	}

	public void setSlow() {
		intakeMotor.setControl(new VoltageOut(IntakeConstants.slowIntakeVolts));
	}

	public void setEject() {
		setMotor(IntakeConstants.ejectSpeed);
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

	public boolean getIntakeIr() {
		return intakeIrValue;
	}

	@Override
	public void close() {
		intakeMotor.close();
	}

	@Override
	public void periodic() {
		double voltage = intakeIrSensor.getVoltage();
		intakeIrValue = voltage > Constants.irSensorThreshold;
		SmartDashboard.putBoolean("INTAKE IR", intakeIrValue);
		SmartDashboard.putNumber("INTAKE SPEED", intakeMotor.getVelocity().getValueAsDouble());
	}
}
