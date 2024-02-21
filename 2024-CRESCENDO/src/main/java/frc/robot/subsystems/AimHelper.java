package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

public class AimHelper {

    /**
     * Returns a double that is the ideal pivot angle for however far away we are from the target.
     * @return A double (in degrees) that is best for our current position. This is from the HashMap in Constants.java.ShooterConstants.rotationLookupTable
     */
    public static AimOutputContainer calculateAimLookupTable(Pose2d robotPose, boolean isBlueSide) {
        AimOutputContainer output = new AimOutputContainer();

        // based on alliance, choose red or blue side speaker as target
        // then move 8ish inches away from april tag into the field (based on alliance, -8 or +8)
        // ^ because april tag is nested outside perimeter (-1.5 inches), we need to move our target goal
        // to be the actual center of the speaker hood (approximated 8)
        Pose2d speakerPosition = (isBlueSide ? AprilTags.BlueSpeakerCenter.value.getPose2d() : AprilTags.RedSpeakerCenter.value.getPose2d()) 
                                    .transformBy(new Transform2d((isBlueSide ? AimConstants.aprilTagToHoodGoal : -AimConstants.aprilTagToHoodGoal), 0, Rotation2d.fromDegrees(0)));

        // calculate distance to speaker
        double robotDistance = speakerPosition.relativeTo(robotPose).getTranslation().getNorm();

        double shooterRotation = 0.0;
        double bestRotationError = Double.POSITIVE_INFINITY;

        for (double currentDistance : Constants.ShooterConstants.rotationLookupTable.keySet()) {
            double currentError = Math.abs(robotDistance - currentDistance);
            // what i want to do here is find the key that has the smallest error from the current distance.
            if (currentError < bestRotationError) {
                shooterRotation = Constants.ShooterConstants.rotationLookupTable.get(currentDistance);
                bestRotationError = currentError;
            }
        }

        double shooterSpeed = 0.0;
        double bestSpeedError = Double.POSITIVE_INFINITY;

        for (double currentDistance : Constants.ShooterConstants.speedLookupTable.keySet()) {
            double currentError = Math.abs(robotDistance - currentDistance);
            // what i want to do here is find the key that has the smallest error from the current distance.
            if (currentError < bestSpeedError) {
                shooterSpeed = Constants.ShooterConstants.speedLookupTable.get(currentDistance);
                bestSpeedError = currentError;
            }
        }
        
        output.setPivotAngle(shooterRotation);
        output.setShooterVelocity(shooterSpeed);
        output.setElevatorHeight(0.0);
        output.setDrivetrainRotation(speakerPosition.getTranslation().minus(robotPose.getTranslation()).getAngle());
        output.setIsInRange(robotDistance < AimConstants.maxRange); 

        return output;

    }

    public static AimOutputContainer calculateAimWithMath(Pose2d robotPosition, ChassisSpeeds robotSpeed, boolean isBlueSide) {
        AimOutputContainer output = new AimOutputContainer(); // basic storage container for holding robot rotations, elevator heights, pivot rotations, and shooter velocities

        double yawGain = AimConstants.yawMomentumGain; // multiply velocity parallel to goal by this gain, and add it to any drivetrain output
        double pitchGain = AimConstants.pitchMomemtumGain; // multiply velocity perpendicular to goal by this gain, and add it to any pivot output

        double velocityParallel = robotSpeed.vxMetersPerSecond; // speed parallel to target - allows yaw momentum compensation
        double velocityPerpendicular = robotSpeed.vyMetersPerSecond; // speed perpendicular to target - allows pitch momentum compensation
        double yawAdd = velocityParallel * yawGain; // adds gains to speeds (which are then added to final yaw or pitch output)
        double pitchAdd = velocityPerpendicular * pitchGain; // adds gains to speeds (which are then added to final yaw or pitch output)


        // based on alliance, choose red or blue side speaker as target
        // then move 8ish inches away from april tag into the field (based on alliance, -8 or +8)
        // ^ because april tag is nested outside perimeter (-1.5 inches), we need to move our target goal
        // to be the actual center of the speaker hood (approximated 8)
        Pose2d speakerPosition = (isBlueSide ? AprilTags.BlueSpeakerCenter.value.getPose2d() : AprilTags.RedSpeakerCenter.value.getPose2d()) 
                                    .transformBy(new Transform2d((isBlueSide ? AimConstants.aprilTagToHoodGoal : -AimConstants.aprilTagToHoodGoal), 0, Rotation2d.fromDegrees(0)));  
        
        // calculate position relative to the speaker for drivetrain.
        Pose2d speakerRelative = speakerPosition.relativeTo(robotPosition);

        // set your final drivetrain output goal based on relative angle difference from speaker - also adds the yaw add gain
        Rotation2d drivetrainGoalRotation = speakerPosition.getTranslation().minus(robotPosition.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(yawAdd));

        // defining constants for physics
        double velocity = ShooterConstants.maxShootSpeed * Units.inchesToMeters(1.5) * Math.PI + AimConstants.compressionAdder;
        double gravity = 9.81;

        // distance to target -> hypotenuous of Translation2d
        double distanceToTarget = speakerRelative.getTranslation().getNorm();

        // height to target -> height of speaker - resting height of elevator
        double heightToTarget = AimConstants.speakerHeight - AimConstants.elevatorHeightFromFloorAtRest;

        // As you get closer, pivot will want to pivot so far that it hits the swerve module and intake gaurd and stuff. 
        // if distance farther than threshold, elevator can remain at bottom as pivot wont pivot as much
        // if distance too close, raise elevator till it will clear, then adjust height and distance from target to 
        // compensate for the movement of the launch pivot. (using sin and cos of elevator tilt.)
        double elevatorHeight = 0.0;
        if (distanceToTarget <= AimConstants.minimumDistanceToNotBreakRobot) {
            elevatorHeight = AimConstants.clearanceHeight;
            heightToTarget -= AimConstants.clearanceHeight * Math.sin(ElevatorConstants.elevatorTilt);
            distanceToTarget += AimConstants.clearanceHeight * Math.cos(ElevatorConstants.elevatorTilt);
        }

        SmartDashboard.putNumber("HEIGHT", heightToTarget);
        SmartDashboard.putNumber("DISTANCE", distanceToTarget);
        SmartDashboard.putNumber("VELO", velocity);

        // tan^-1((v^2 - sqrt(v^4 - g (gx^2 + 2yv^2))) / (gx))
        // equation to calculate theta given velocity, x distance, and y distance from target
        double theta = Math.atan((Math.pow(velocity, 2) - Math.sqrt(Math.pow(velocity, 4) - gravity * (gravity * Math.pow(distanceToTarget, 2) + 2 * heightToTarget * Math.pow(velocity, 2)))) / (gravity * distanceToTarget));
        SmartDashboard.putNumber("THETA", Math.toDegrees(theta));

        // set shooter angle by double checking theta by plugging back into equation (eliminates complex numbers (i think D:)). 
        // regardless of if it works or not, it allows a tolerance to be put on accuracy which is good.
        boolean passesTesting = false;

        // plugs angle back in, and solves for the distance it will go
        double initialVelocityX = velocity * Math.cos(theta);
        double initialVelocityY = velocity * Math.sin(theta);
        double timeOfFlight = distanceToTarget / initialVelocityX;
        double verticalDistanceTraveled =initialVelocityY * timeOfFlight - 0.5 * gravity * timeOfFlight * timeOfFlight;
        
        // if predicted height traveled is within tolerance, allows the value to filter back
        passesTesting = (verticalDistanceTraveled - heightToTarget < AimConstants.rangeTolerance);

        // line of best fitted the gain using Desmos :)
        double pivotDragGain = distanceToTarget * AimConstants.dragPitchGainSlope + AimConstants.dragPitchGainYIntercept;

        // sets shooter angle finally. if doesnt pass testing, keep old shooter angle, else change it - returns in radians
        // also adds the pitch gain, and then multiplies by the drag gain.
        double pivotAngleRadians = passesTesting ? (theta + pitchAdd) * pivotDragGain : 0.0;

        // because output is technically from floor, gotta convert into output from our zero
        output.setPivotAngleFromDegreesFromFloor(Math.toDegrees(pivotAngleRadians));
        output.setShooterVelocity(velocity);
        output.setIsInRange(distanceToTarget < AimConstants.maxRange);
        output.setDrivetrainRotation(drivetrainGoalRotation);
        output.setElevatorHeight(elevatorHeight);

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
