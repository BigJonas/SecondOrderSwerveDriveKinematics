// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SO_SwerveDriveKinematics_Test {
    private final int MODULE_NUM = 4;

    private final Translation2d FRONT_LEFT     = new Translation2d(+1, +1);
    private final Translation2d FRONT_RIGHT    = new Translation2d(+1, -1); 
    private final Translation2d BACK_LEFT      = new Translation2d(-1, +1);
    private final Translation2d BACK_RIGHT     = new Translation2d(-1, -1);

    private final SO_ChassisSpeeds UNIT_CHASSIS_SPEEDS = new SO_ChassisSpeeds(1, 1, 1, 1, 1, 1);

    private final SO_SwerveModuleState_Test[] UNIT_MODULE_STATES = {
        new SO_SwerveModuleState_Test(1, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new SO_SwerveModuleState_Test(1, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new SO_SwerveModuleState_Test(1, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new SO_SwerveModuleState_Test(1, 0, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    };

    private final SO_SwerveDriveKinematics mSOKinematics;
    private final SwerveDriveKinematics mFOKinematics;

    public SO_SwerveDriveKinematics_Test() {
        mSOKinematics = new SO_SwerveDriveKinematics(
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        );
        mFOKinematics = new SwerveDriveKinematics(
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        );
    }
    
    /**
     * Test if {@link frc.robot.util.SO_SwerveDriveKinematics#toModuleStates(SO_ChassisSpeeds, Translation2d)} and
     * {@link frc.robot.util.SO_SwerveDriveKinematics#toChassisSpeeds(SO_SwerveModuleState...)} are direct inverses
     * Don't know if the problem is with the Foward or Inverse kinematics
     */
    @Test
    void testIsInversable() {
        SO_SwerveModuleState[] moduleStates = mSOKinematics.toSwerveModuleStates(UNIT_CHASSIS_SPEEDS);
        System.out.println(mSOKinematics.toChassisSpeeds(moduleStates));

        assertTrue(chassisSpeedsEqual(UNIT_CHASSIS_SPEEDS, mSOKinematics.toChassisSpeeds(moduleStates)));
    }

    /**
     * Test if the first order of the SO_SwerveDriveKinematics inverse kinematics 
     * is the same as SwerveDriveKinematics inverse kinematics
     */
    @Test
    void testInverse() {
        SO_SwerveModuleState[] so_moduleStates = mSOKinematics.toSwerveModuleStates(UNIT_CHASSIS_SPEEDS); 
        SwerveModuleState[] fo_moduleStates = mFOKinematics.toSwerveModuleStates(UNIT_CHASSIS_SPEEDS);
        for (int i = 0; i < MODULE_NUM; i++) {
            assertTrue(fo_moduleStates[i].compareTo(so_moduleStates[i].asFirstOrder()) == 0);
        }
    }

    /**
     * Test if the first order of the SO_SwerveDriveKinematics foward kinematcs
     * is the same as SwerveDriveKinematics foward kinematics
     */
    @Test
    void testFoward() {
        ChassisSpeeds so_chassisSpeeds = mSOKinematics.toChassisSpeeds(UNIT_MODULE_STATES).asFirstOrder();
        ChassisSpeeds fo_chassisSpeeds = mFOKinematics.toChassisSpeeds(UNIT_MODULE_STATES);
        assertTrue(chassisSpeedsEqual(so_chassisSpeeds, fo_chassisSpeeds));
    }

    // Why doesnt WPI lib have a compare for chassis speeds
    private boolean chassisSpeedsEqual(ChassisSpeeds a, ChassisSpeeds b) {
        if (
            a.vxMetersPerSecond != b.vxMetersPerSecond || // Agony
            a.vyMetersPerSecond != b.vyMetersPerSecond || 
            a.omegaRadiansPerSecond != b.omegaRadiansPerSecond) {
                return false;
        }
        return true;
    }

}
