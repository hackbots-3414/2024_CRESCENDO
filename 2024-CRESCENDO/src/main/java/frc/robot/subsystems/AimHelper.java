package frc.robot.subsystems; 

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

public class AimHelper {
    public enum AimStrategies {LOOKUP, MATH, PHYSICS;}
    
    public static AimOutputContainer getAimOutputs(CommandSwerveDrivetrain drivetrain, boolean isBlueSide, AimStrategies strategy) {
        Pose2d speakerPose = isBlueSide ? AimConstants.blueSpeakerPos : AimConstants.redSpeakerPos;
        Pose2d drivetrainPose = drivetrain.getPose();
        // Pose2d speakerPoseForMath = speakerPose.transformBy(new Transform2d((isBlueSide ? -AimConstants.aprilTagToHoodGoal : AimConstants.aprilTagToHoodGoal), 0, Rotation2d.fromDegrees(0)));

        double robotDistance = speakerPose.relativeTo(drivetrainPose).getTranslation().getNorm();

        AimOutputContainer output = new AimOutputContainer();
        switch (strategy) {
            case LOOKUP:
                output = useLookupTable(robotDistance);
                break;
            case MATH:
                output = useMath(robotDistance);
                break;
            case PHYSICS:
                output = usePhysics(robotDistance);
                break;
        }

        output.setDrivetrainRotation(speakerPose.getTranslation().minus(drivetrainPose.getTranslation()).getAngle());
        output.setIsInRange(robotDistance < AimConstants.maxRange); 
        output.setElevatorHeight(0.0);

        return output;
    }

    private static AimOutputContainer useLookupTable(double distanceToTarget) {
        AimOutputContainer output = new AimOutputContainer();

        Double lowerDistance = null, higherDistance = null;

        Map<Double, Double> rotationMap = ShooterConstants.rotationLookupTable;
        Map<Double, Double> speedMap = ShooterConstants.speedLookupTable;
        for (Double key : rotationMap.keySet()) {
            lowerDistance = (key <= distanceToTarget && (lowerDistance == null || key > lowerDistance)) ? key : lowerDistance;
            higherDistance = (key >= distanceToTarget && (higherDistance == null || key < higherDistance)) ? key : higherDistance;
        }

        // If exact distance is found, return the corresponding value
        if (lowerDistance != null && lowerDistance == distanceToTarget) {
            output.setShooterVelocity(speedMap.get(lowerDistance));
            output.setPivotAngle(rotationMap.get(lowerDistance));
        } else {
            // If no lower or higher distance is found, return 0
            if (lowerDistance == null || higherDistance == null) {
                output.setShooterVelocity(0);
                output.setPivotAngle(0);
            } else {
                double location = (distanceToTarget - lowerDistance) / (higherDistance - lowerDistance);
                output.setShooterVelocity(MathUtil.interpolate(speedMap.get(lowerDistance), speedMap.get(higherDistance), location));
                output.setPivotAngle(MathUtil.interpolate(rotationMap.get(lowerDistance), rotationMap.get(higherDistance), location));
            }
        }

        return output;
    }

    private static AimOutputContainer useMath(double distanceToTarget) {
        AimOutputContainer output = new AimOutputContainer();    
        output.setPivotAngleFromRadFromFloor(Math.atan2(AimConstants.speakerHeightMinusElevatorRaise, distanceToTarget));
        output.setShooterVelocity(ShooterConstants.maxShootSpeed);
        return output;
    }

    private static AimOutputContainer usePhysics(double distanceToTarget) {
        AimOutputContainer output = new AimOutputContainer();

        double theta = Math.atan((Math.pow(AimConstants.velocity, 2) - Math.sqrt(Math.pow(AimConstants.velocity, 4) - AimConstants.gravity * (AimConstants.gravity * Math.pow(distanceToTarget, 2) + 2 * AimConstants.speakerHeightMinusElevatorRaise * Math.pow(AimConstants.velocity, 2)))) / (AimConstants.gravity * distanceToTarget));
        double pivotDragGain = distanceToTarget * AimConstants.dragPitchGainSlope + AimConstants.dragPitchGainYIntercept;
        
        output.setPivotAngleFromRadFromFloor(theta * pivotDragGain);
        output.setPivotAngle(MathUtil.clamp(output.getPivotAngle(), 0.0, 0.072));
        output.setShooterVelocity(ShooterConstants.maxShootSpeed);

        return output;
    }

    public static class AimOutputContainer {
        private double elevatorHeight; // output shaft rotations
        private double pivotAngle; // rotations of pivot
        private Rotation2d drivetrainRotation; 
        private double shooterVelocity;
        private boolean isInRange;

        public AimOutputContainer() {}

        public double getElevatorHeight() {
            return elevatorHeight;
        }

        public double getElevatorHeightInches() {
            return elevatorHeight * Math.PI * ElevatorConstants.gearDiameter;
        }

        public void setElevatorHeight(double shaftRotations) {
            this.elevatorHeight = shaftRotations;
        }

        public void setElevatorHeightFromInches(double inches) {
            this.elevatorHeight = (inches / (Math.PI * ElevatorConstants.gearDiameter));
        }

        public double getPivotAngle() {
            return pivotAngle;
        }

        public double getPivotAngleDegrees() {
            return pivotAngle * 360;
        }

        public double getPivotAngleDegreesFromZero() {
            return (pivotAngle * 360) + Math.toDegrees(PivotConstants.radiansAtZero);
        }

        public void setPivotAngle(double pivotRotationsFromZero) {
            this.pivotAngle = pivotRotationsFromZero;
        }

        public void setPivotAngleFromDegrees(double degreesFromZero) {
            this.pivotAngle = degreesFromZero / 360;
        }

        public void setPivotAngleFromDegreesFromFloor(double degreesFromFloor) {
            this.pivotAngle = (degreesFromFloor - Math.toDegrees(PivotConstants.radiansAtZero)) / 360;
        }

        public void setPivotAngleFromRadFromFloor(double radiansFromFloor) {
            this.pivotAngle = (radiansFromFloor - PivotConstants.radiansAtZero) / (Math.PI * 2);
        }

        public Rotation2d getDrivetrainRotation() {
            return drivetrainRotation;
        }

        public void setDrivetrainRotation(Rotation2d rotation2d) {
            this.drivetrainRotation = rotation2d;
        }

        public double getShooterVelocity() {
            return shooterVelocity;
        }

        public void setShooterVelocity(double revolutionsPerSecond) {
            this.shooterVelocity = revolutionsPerSecond;
        }

        public void setShooterVelocityFromPercentMax(double percentOfMax) {
            this.shooterVelocity = (percentOfMax * (ShooterConstants.maxShootSpeed - ShooterConstants.minShootSpeed)) + ShooterConstants.minShootSpeed;
        }

        public boolean getIsInRange() {
            return isInRange;
        }

        public void setIsInRange(boolean isInRange) {
            this.isInRange = isInRange;
        }
    }
}
