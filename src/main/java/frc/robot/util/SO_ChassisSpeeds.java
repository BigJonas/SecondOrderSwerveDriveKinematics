// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents second order speeds of a robot chassis 
 * See ChassisSpeeds class for better documentation
 * {@link edu.wpi.first.math.kinematics.ChassisSpeeds}
 */
public class SO_ChassisSpeeds {
    public double vxMetersPerSecond;
    public double vyMetersPerSecond;
    public double omegaRadiansPerSecond;
    public double axMetersPerSecondSquard;
    public double ayMetersPerSecondSquard;
    public double alphaRadiansPerSecond;

    /**
     * Constructs a Second Order Chassis Speed object
     * @param vxMetersPerSecond Forward Velocity
     * @param vyMetersPerSecond Sideways Velocity
     * @param omegaRadiansPerSecond Angular Velocity
     * @param axMetersPerSecondSquard Forward Acceleration
     * @param ayMetersPerSecondSquard Sideways Acceleration
     * @param alphaRadiansPerSecond Angular Acceleration
     */
    public SO_ChassisSpeeds(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
        double axMetersPerSecondSquard, double ayMetersPerSecondSquard, double alphaRadiansPerSecond) {
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.omegaRadiansPerSecond = omegaRadiansPerSecond;
            this.axMetersPerSecondSquard = axMetersPerSecondSquard;
            this.ayMetersPerSecondSquard = ayMetersPerSecondSquard;
            this.alphaRadiansPerSecond = alphaRadiansPerSecond;
    }

  /**
   * Converts field relative chassis speeds into robot relative chassis speeds
   * @param fieldRelativeSpeeds Desired field relative chassis speeds 
   * @param robotAngle Current robot angle
   * @return Desired robot relative chassis speeds
   */
    public static SO_ChassisSpeeds fromFieldRelativeSpeeds(SO_ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
        return new SO_ChassisSpeeds(
            +fieldRelativeSpeeds.vxMetersPerSecond * robotAngle.getCos() + fieldRelativeSpeeds.vyMetersPerSecond * robotAngle.getSin(),
            -fieldRelativeSpeeds.vyMetersPerSecond * robotAngle.getSin() + fieldRelativeSpeeds.vxMetersPerSecond * robotAngle.getCos(), 
            fieldRelativeSpeeds.omegaRadiansPerSecond, 
            +fieldRelativeSpeeds.axMetersPerSecondSquard * robotAngle.getCos() + fieldRelativeSpeeds.vyMetersPerSecond * robotAngle.getSin(), 
            -fieldRelativeSpeeds.ayMetersPerSecondSquard * robotAngle.getSin() + fieldRelativeSpeeds.vxMetersPerSecond * robotAngle.getCos(), 
            fieldRelativeSpeeds.alphaRadiansPerSecond
        );
    }
}
