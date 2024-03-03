package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.Robot;
import frc.robot.Telemetry;
import frc.robot.commands.AutonCommands.AutoScoreCommand;
import frc.robot.commands.AutonCommands.RevShooterCommand;
import frc.robot.commands.AutonCommands.ShootAfterRevCommand;
import frc.robot.commands.BaseSubsystemCommands.AimCommand;
import frc.robot.commands.BaseSubsystemCommands.ElevatorCommand;
import frc.robot.commands.BaseSubsystemCommands.ElevatorCommand.ElevatorPresets;
import frc.robot.commands.BaseSubsystemCommands.IntakeCommand;
import frc.robot.commands.BaseSubsystemCommands.ShooterCommand;
import frc.robot.commands.ComboCommands.AmpScoreCommand;
import frc.robot.commands.ComboCommands.ResetElevatorCommand;
import frc.robot.commands.ComboCommands.StealRingCommand;
import frc.robot.commands.ComboCommands.TrapScoreCommand;
import frc.robot.commands.ManualCommands.ManualElevatorCommand;
import frc.robot.commands.ManualCommands.ManualPivotCommand;
import frc.robot.commands.ManualCommands.ManualWinchCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AimHelper.AimOutputContainer;
import frc.robot.subsystems.AimHelper.AimStrategies;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;

public class SubsystemManager extends SubsystemBase {
	private AprilTagVision aprilTagVision;
	private static SubsystemManager me = null;
	PowerDistribution pdp = new PowerDistribution(PDPConstants.pdp, ModuleType.kRev);
	List<SubsystemBase> subsystems = new ArrayList<>();

	Supplier<Alliance> allianceSupplier = () -> DriverStation.getAlliance().get();

	private boolean noteOnBoard = false;
	private boolean aimReady = false;

	
	// DRIVETRAIN
	CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
	FieldCentric driveRequest = new SwerveRequest.FieldCentric()
			// .withDeadband(Constants.SwerveConstants.maxDriveVelocity * 0.05)
			// .withRotationalDeadband(Constants.SwerveConstants.maxAngleVelocity * 0.05)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	SwerveDriveBrake brakeRequest = new SwerveDriveBrake();
	PointWheelsAt pointRequest = new PointWheelsAt();
	Telemetry logger = new Telemetry();

	ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
			.withDriveRequestType(DriveRequestType.Velocity);
	HashMap<String, Command> eventMarkers = new HashMap<>();


	// CURRENT LIMITING
	double elevatorCurrent = 0;
	double intakeCurrent = 0;
	double shooterCurrent = 0;
	double shooterPivotCurrent = 0;
	double transportCurrent = 0;
	double winchCurrent = 0;

	double inputCurrent = 18; // 18 AMP HOURS
	double runTimeHours = 0.05; // 3 MINUTES
	double coprocessorsAmpRating = 3 * 2 * runTimeHours; // 3 AMP HOURS for runTimeHours per coprocessor
	double availableCurrent = inputCurrent - coprocessorsAmpRating;

	int periodicRuns = 0;

	// SUBSYSTEMS
	Elevator elevator = new Elevator();
	Intake intake = new Intake();
	Shooter shooter = new Shooter();
	ShooterPivot shooterPivot = new ShooterPivot();
	Transport transport = new Transport();
	NoteFinder noteFinder = new NoteFinder();
	Winch winch = new Winch();
	LedSubsystem ledSubsystem = new LedSubsystem(this::getNoteOnBoard, this::getIsInRange, this::getAimIsReady);

	public Intake getIntake() {return intake;}
	public Shooter getShooter() {return shooter;}
	public ShooterPivot getShooterPivot() {return shooterPivot;}
	public Transport getTransport() {return transport;}
	public Elevator getElevator() {return elevator;}
	public NoteFinder getNoteFinder() {return noteFinder;}
	public Winch getWinch() {return winch;}
	public LedSubsystem getLedSubsystem() {return ledSubsystem;}
	public AprilTagVision getAprilTagVision() {return aprilTagVision;}

	private SubsystemManager() {
		configurePathPlanner();

		if (Constants.VisionConstants.USE_VISION) {
			if (Robot.isReal()) {
            	aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVision());
			} else {
				aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVision());
				// aprilTagVision = new AprilTagVision(new AprilTagVisionIOPhotonVisionSIM(drivetrain::getCurrentPose2d));
			}
            aprilTagVision.setDataInterfaces(drivetrain::addVisionData);
        }
	}

	public static synchronized SubsystemManager getInstance() {
		if (me == null) {
			me = new SubsystemManager();
		}
		return me;
	}

  @Override
	public void periodic() {
		elevatorCurrent = pdp.getCurrent(PDPConstants.elevator);
		intakeCurrent = pdp.getCurrent(PDPConstants.intake);
		shooterPivotCurrent = pdp.getCurrent(PDPConstants.pivot);
		shooterCurrent = pdp.getCurrent(PDPConstants.shooterLeft) + pdp.getCurrent(PDPConstants.shooterRight);
		transportCurrent = pdp.getCurrent(PDPConstants.transport);
		winchCurrent = pdp.getCurrent(PDPConstants.winchLeft) + pdp.getCurrent(PDPConstants.winchRight);

		// dampenDrivetrain();
	}

	private void dampenDrivetrain() {
		// (Ah Available - Ah Being Used) / Ah to Amps conversion / 4 motors to distribute over
		double supplyLimitDrivetrain = ((availableCurrent / runTimeHours
				- (elevatorCurrent + intakeCurrent + shooterPivotCurrent + shooterCurrent + transportCurrent))) / 4.0; 
		supplyLimitDrivetrain = supplyLimitDrivetrain > SwerveConstants.driveSupplyCurrentLimit ? SwerveConstants.driveSupplyCurrentLimit : supplyLimitDrivetrain;
		drivetrain.setCurrentLimit(supplyLimitDrivetrain);
	}

	// DRIVETRAIN COMMANDS
	public void configureDriveDefaults(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
		drivetrain.setDefaultCommand(
				drivetrain.applyRequest(
						() -> driveRequest.withVelocityX(-x.get() * Constants.SwerveConstants.maxDriveVelocity)
								.withVelocityY(-y.get() * Constants.SwerveConstants.maxDriveVelocity)
								.withRotationalRate(-turn.get() * Constants.SwerveConstants.maxAngleVelocity)));
	}
	public Command makeBrakeCommand() {
		return drivetrain.applyRequest(() -> brakeRequest);
	}
	public Command makePointCommand(double x, double y) {
		return drivetrain.applyRequest(() -> pointRequest.withModuleDirection(new Rotation2d(-x, -y)));
	}
	public Command makeResetCommand() {
		return drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
	}
	public Command resetAfterAuton() {
		return drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(allianceSupplier.get() == Alliance.Blue ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)));
	}
	public Command makeShellyCommand(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
		Command shellyCommand = drivetrain
				.applyRequest(() -> driveRequest.withVelocityX(-y.get() * Constants.SwerveConstants.shellyDriveVelocity)
						.withVelocityY(-x.get() * Constants.SwerveConstants.shellyDriveVelocity)
						.withRotationalRate(-turn.get() * Constants.SwerveConstants.shellyAngleVelocity));
		shellyCommand.addRequirements(drivetrain);
		return shellyCommand;
	}
	public Command resetAtPose2d(Pose2d pose) {
		return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(pose));
	}
	public void telemeterize() {
		drivetrain.registerTelemetry(logger::telemeterize);
	}


	// ELEVATOR COMMANDS
	public Command makeElevatorCommand(ElevatorPresets preset) {
		return new ElevatorCommand(elevator, shooterPivot, preset);
	}
	public Command makeResetElevatorCommand() {
		return new ResetElevatorCommand(elevator, shooterPivot);
	}
	public Command makeManualElevatorCommand(boolean isUp) {
		return new ManualElevatorCommand(elevator,
				isUp ? ElevatorConstants.elevatorManualUpSpeed : ElevatorConstants.elevatorManualDownSpeed);
	}
	public Command elevatorNeutralMode(NeutralModeValue neutralMode) {
		return new InstantCommand(() -> elevator.setNeutralMode(neutralMode));
	}


	// WINCH COMMANDS
	public Command makeManualWinchCommand(boolean isUp) {
		return new ManualWinchCommand(winch,
				isUp ? WinchConstants.winchManualUpSpeed : WinchConstants.winchManualDownSpeed);
	}


	// PIVOT COMMANDS
	public Command makeManualPivotCommand(boolean isUp) {
		return new ManualPivotCommand(shooterPivot,
				isUp ? PivotConstants.pivotManualUpSpeed : PivotConstants.pivotManualDownSpeed);
	}


	// SHOOTER COMMANDS
	public Command makeRevShootCommand(double velocity) {
		return new RevShooterCommand(shooter, transport, velocity);
	}
	public Command makeShootAfterRevCommand(double velocity) {
		return new ShootAfterRevCommand(shooter, transport, velocity, this::setNoteOnBoard).withTimeout(0.35);
	}
	public Command makeShootCommand() {
		return new ShooterCommand(shooter, transport, ShooterConstants.shootVelo, this::setNoteOnBoard);
	}


	// INTAKE COMMANDS
	public Command makeIntakeCommand() {
		return new IntakeCommand(transport, intake, elevator, shooterPivot, Constants.IntakeConstants.intakeSpeed, Constants.TransportConstants.transportSpeed, this::setNoteOnBoard);
	}


	// PRESETS COMMANDS
	public Command makeAmpScoreCommand() {
		return new AmpScoreCommand(transport, elevator, shooter, shooterPivot);
	}
	public Command makeTrapScoreCommand() {
		return new TrapScoreCommand(transport, elevator, shooter, shooterPivot);
	}
	public Command makeSubwooferShootCommand() {
		return new SequentialCommandGroup(makeElevatorCommand(ElevatorPresets.SUBWOOFER),
				new ShooterCommand(shooter, transport, ShooterConstants.minShootSpeed, this::setNoteOnBoard));
	}
	public Command makeSubwooferRevvingCommand() {
		return new SequentialCommandGroup(makeElevatorCommand(ElevatorPresets.SUBWOOFER),
				makeRevShootCommand(ShooterConstants.minShootSpeed));
	}


	// AIMING COMMANDS
	public Command makeAutoAimCommand(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
		return new AimCommand(elevator, shooterPivot, drivetrain, x, y, turn, allianceSupplier, this::setAimReady);
	}
	public AimOutputContainer getAimOutputContainer() {
        return AimHelper.getAimOutputs(drivetrain, allianceSupplier.get() == Alliance.Blue, AimStrategies.LOOKUP); // BASIC MATH
	}
	public void setAimReady(boolean isReady) {
		this.aimReady = isReady;
	}


	// LED GETTERS
	public boolean getAimIsReady() {
		return aimReady;
	}
	public boolean getIsInRange() {
		return drivetrain.isInRange();
	}
	public boolean getNoteOnBoard() {
		return noteOnBoard;
	}
	public void setNoteOnBoard(boolean noteOnBoard) {
		this.noteOnBoard = noteOnBoard;
	}


	// AUTON COMMANDS
	public Command makeStealRingCommand() {
		return new StealRingCommand(shooter, makeIntakeCommand(), elevator, shooterPivot);
	}
	public Optional<Rotation2d> getRotationTargetOverride() {
		if (noteOnBoard) {
			return Optional.of(getAimOutputContainer().getDrivetrainRotation());
		} else {
			return Optional.empty();
		}
	}
	public Command makeAutonEverythingCommand() {
		return /*makeIntakeCommand().andThen(*/new AutoScoreCommand(elevator, shooterPivot, shooter, transport, this::getAimOutputContainer, this::setNoteOnBoard).withTimeout(1.5);
	}


	public Command makeTestingCommand() {
		SequentialCommandGroup commands = new SequentialCommandGroup();
		commands.addCommands(makeElevatorCommand(ElevatorPresets.AMP).withTimeout(2),
				makeElevatorCommand(ElevatorPresets.STOW).withTimeout(2),
				makeIntakeCommand().withTimeout(2),
				makeShootCommand().withTimeout(2),
				new ManualWinchCommand(winch, 0.1).withTimeout(2),
				new ManualWinchCommand(winch, -0.1).withTimeout(2));

		return commands;
	}

	private void configurePathPlanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : drivetrain.moduleLocations()) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		SmartDashboard.putNumber("driveabase radius", driveBaseRadius);
	
		eventMarkers.put("Subwoofer", makeElevatorCommand(ElevatorPresets.SUBWOOFER).andThen(makeShootAfterRevCommand(ShooterConstants.minShootSpeed)).andThen(makeElevatorCommand(ElevatorPresets.STOW)));
		eventMarkers.put("Intake", makeIntakeCommand().andThen(makeSubwooferRevvingCommand())); 
		eventMarkers.put("IntakeThenSubwooferPreset", makeIntakeCommand().andThen(makeSubwooferRevvingCommand()));
		eventMarkers.put("StealRings", makeStealRingCommand());
		eventMarkers.put("Stow", makeElevatorCommand(ElevatorPresets.STOW));
		
		eventMarkers.put("ShootAnywhere", makeAutonEverythingCommand());

		NamedCommands.registerCommands(eventMarkers);

		AutoBuilder.configureHolonomic(
				() -> drivetrain.getState().Pose, // CurrentPose Supplier
				drivetrain::seedFieldRelative, // PoseSetter Consumer
				drivetrain::getCurrentRobotChassisSpeeds,
				(speeds) -> drivetrain.setControl(autoRequest.withSpeeds(speeds)), // ChassisSpeeds Consumer
				new HolonomicPathFollowerConfig(
						new PIDConstants(SwerveConstants.kPDrive, SwerveConstants.kIDrive, SwerveConstants.kDDrive),
						new PIDConstants(SwerveConstants.kPSteer, SwerveConstants.kISteer, SwerveConstants.kDSteer),
						TunerConstants.kSpeedAt12VoltsMps,
						driveBaseRadius,
						new ReplanningConfig(true, true)),
				() -> {
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				},
				drivetrain); // Subsystem for requirements

		// PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
	}
}