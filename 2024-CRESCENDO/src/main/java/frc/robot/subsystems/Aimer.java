package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class Aimer {

    /**
     * Returns a double that is the ideal pivot angle for however far away we are from the target.
     * @return A double (in degrees) that is best for our current position. This is from the HashMap in Constants.java.ShooterConstants.rotationLookupTable
     */
    public static double calculateAim(Pose3d robotPose, Pose3d goalPose) {
        // calculate the top-down rotation angle
        double robotDistance = robotPose.toPose2d().getTranslation().getDistance(goalPose.toPose2d().getTranslation());

        double shooterRotation = 0.0;
        double bestError = Double.POSITIVE_INFINITY;

        for (double currentDistance : Constants.ShooterConstants.rotationLookupTable.keySet()) {
            double currentError = Math.abs(robotDistance - currentDistance);
            // what i want to do here is find the key that has the smallest error from the current distance.
            if (currentError < bestError) {
                shooterRotation = Constants.ShooterConstants.rotationLookupTable.get(currentDistance);
                bestError = currentError;
            }
        }

        return shooterRotation;
    }
}
