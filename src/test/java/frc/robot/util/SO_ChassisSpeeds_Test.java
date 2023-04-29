// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class SO_ChassisSpeeds_Test extends SO_ChassisSpeeds {
    public SO_ChassisSpeeds_Test(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
        double axMetersPerSecondSquared, double ayMetersPerSecondSquared, double alphaRadiansPerSecond) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, axMetersPerSecondSquared, ayMetersPerSecondSquared, alphaRadiansPerSecond);
    }

    public ChassisSpeeds asFirstOrder() {
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}
