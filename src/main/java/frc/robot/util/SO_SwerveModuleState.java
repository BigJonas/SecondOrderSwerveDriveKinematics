// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a Swerve Module State that modules both Velocity and Acceleration for Speed and Position and Velocity for Angle
 * See SwerveModuleState for better documentation
 * {@link edu.wpi.first.math.kinematics.SwerveModuleState}
 */
public class SO_SwerveModuleState {
    public double speedMetersPerSecond;
    public double accelMetersPerSecondSquard;

    public Rotation2d anglePos = Rotation2d.fromRadians(0);
    public Rotation2d angleVel = Rotation2d.fromRadians(0);

    /**
     * Constructs an empty Serve Module State
     */
    public SO_SwerveModuleState() {}

    /**
     * Constructs a Second Order Swerve Module State
     * @param speedMetersPerSecond Speed of the module 
     * @param accelMetersPerSecondSquard Acceleration of the module
     * @param anglePos Positional Angle of the module
     * @param angleVel Angular Velocity of the module
     */
    public SO_SwerveModuleState(
        double speedMetersPerSecond, double accelMetersPerSecondSquard,
        Rotation2d anglePos, Rotation2d angleVel) {
            this.speedMetersPerSecond = speedMetersPerSecond;
            this.accelMetersPerSecondSquard = accelMetersPerSecondSquard;
            this.anglePos = anglePos;
            this.angleVel = angleVel;
    }

    /**
     * Optimizes a Swerve Module State so it will travel in the optimal direction
     * @param desiredState Desired state of the module 
     * @param currentAngle The current module angle
     * @return Optimized Swerve Module State
     */
    public static SO_SwerveModuleState optimize(SO_SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.anglePos.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SO_SwerveModuleState(
                -desiredState.speedMetersPerSecond, 
                -desiredState.accelMetersPerSecondSquard, 
                desiredState.anglePos.rotateBy(Rotation2d.fromDegrees(180)), 
                desiredState.angleVel.times(-1) 
            );
        } else {
            return new SO_SwerveModuleState(
                desiredState.speedMetersPerSecond, 
                desiredState.accelMetersPerSecondSquard, 
                desiredState.anglePos, 
                desiredState.angleVel 
            );
        }
    }
}
