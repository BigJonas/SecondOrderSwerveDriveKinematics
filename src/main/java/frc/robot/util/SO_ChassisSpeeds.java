// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Represents second order speeds of a robot chassis 
 * See ChassisSpeeds class for better documentation
 * {@link edu.wpi.first.math.kinematics.ChassisSpeeds}
 */
public class SO_ChassisSpeeds extends ChassisSpeeds {
    // Linear Acceleration in the x-axis
    public double axMetersPerSecondSquared;
    // Linear Acceleration in the y-axis
    public double ayMetersPerSecondSquared;
    // Angular Acceleration 
    public double alphaRadiansPerSecondSquared;

    /**
     * Constructs a Second Order Chassis Speed object
     * @param vxMetersPerSecond Forward Velocity
     * @param vyMetersPerSecond Sideways Velocity
     * @param omegaRadiansPerSecond Angular Velocity
     * @param axMetersPerSecondSquared Forward Acceleration
     * @param ayMetersPerSecondSquared Sideways Acceleration
     * @param alphaRadiansPerSecondSquared Angular Acceleration
     */
    public SO_ChassisSpeeds(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
        double axMetersPerSecondSquared, double ayMetersPerSecondSquared, double alphaRadiansPerSecondSquared) {
            super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
            this.axMetersPerSecondSquared = axMetersPerSecondSquared;
            this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
            this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
    }

    /**
     * Returns the first order components of the ChassisSpeeds
     * @return First order Chassis Speeds
     */
    public ChassisSpeeds asFirstOrder() {
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
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
            +fieldRelativeSpeeds.axMetersPerSecondSquared * robotAngle.getCos() + fieldRelativeSpeeds.vyMetersPerSecond * robotAngle.getSin(), 
            -fieldRelativeSpeeds.ayMetersPerSecondSquared * robotAngle.getSin() + fieldRelativeSpeeds.vxMetersPerSecond * robotAngle.getCos(), 
            fieldRelativeSpeeds.alphaRadiansPerSecondSquared
        );
    }

    @Override
    public String toString() {
      return String.format(
          "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s, Ax: %.2f m/s^2, Ay: %.2f m/s^2, Alpha: %.2f rad/s^2)",
          vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, axMetersPerSecondSquared, ayMetersPerSecondSquared, alphaRadiansPerSecondSquared);
    }
}
