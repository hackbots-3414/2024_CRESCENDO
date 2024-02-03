// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Gamepiece {
    private double angle = 0;
    private double confidence = 0;
    
    public double getAngle() {
        return angle;
    }
    public void setAngle(double angle) {
        this.angle = angle;
    }
    public double getConfidence() {
        return confidence;
    }
    public void setConfidence(double confidence) {
        this.confidence = confidence;
    }

    public String toString() {
        StringBuilder out = new StringBuilder("Gamepiece angle: ").append(angle);
        out.append(", confidence: ").append(confidence);
        return out.toString();
    }
    
}
