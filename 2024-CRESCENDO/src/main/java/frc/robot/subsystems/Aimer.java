// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Aimer {

    public static Rotation3d calculateAim(Pose3d robotPose, Pose3d goalPose) {
        // calculate the top-down rotation angle
        Pose3d relativePose = goalPose.relativeTo(robotPose);
        double robotRotation = relativePose.getRotation().getZ(); // this does not work... please make it work

        double robotDistance = relativePose.toPose2d().getTranslation().getDistance(new Translation2d());

        double shooterRotation = -1.0;
        double bestError = Double.POSITIVE_INFINITY;

        for (double currentDistance : Constants.ShooterConstants.rotationLookupTable.keySet()) {
            double currentError = Math.abs(robotDistance - currentDistance);
            // what i want to do here is find the key that has the smallest error from the current distance.
            if (currentError < bestError) {
                shooterRotation = Constants.ShooterConstants.rotationLookupTable.get(currentDistance);
                bestError = currentError;
            }
        }

        // because the values in our map are in degrees for better readability, we now need to convert the angles to radians.
        shooterRotation *= (Math.PI / 180.0);

        return new Rotation3d(0, shooterRotation, robotRotation);
    }
}
