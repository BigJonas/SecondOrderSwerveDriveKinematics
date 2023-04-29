// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A Swerve Drive Kinematics class that models both acceleration and velocity
 * See SwerveDriveKinematics class for better documentation
 * {@link edu.wpi.first.math.kinematics.SwerveDriveKinematics}
 */
public class SO_SwerveDriveKinematics {
    private final SimpleMatrix mInverseKinematics;
    private final SimpleMatrix mInverseKinematicsPrime;

    private final SimpleMatrix mForwardKinematics;
    private final SimpleMatrix mForwardKinematicsPrime;

    private final int mModuleNum;
    private final Translation2d[] mModules;
    private SO_SwerveModuleState[] mModuleStates;
    private Translation2d mPreviousCenterOfRotation;
    
    /**
     * Creates a 
     * @param wheelMeters
     */
    public SO_SwerveDriveKinematics(Translation2d... wheelMeters) {
        if (wheelMeters.length < 2) {
            throw new IllegalArgumentException("Cannot have less than 2 swerve module !!!!");
        }

        mModuleNum = wheelMeters.length;
        mModules = Arrays.copyOf(wheelMeters, mModuleNum);
        mModuleStates = new SO_SwerveModuleState[mModuleNum];
        Arrays.fill(mModuleStates, new SO_SwerveModuleState());

        mInverseKinematics = new SimpleMatrix(mModuleNum * 2, 3);
        mInverseKinematicsPrime = new SimpleMatrix(mModuleNum * 2, 4);

        mPreviousCenterOfRotation = new Translation2d();

        for (int i = 0; i < mModuleNum; i++) {
            mInverseKinematics.setRow(i * 2 + 0, 0, 
                1, 0, -mModules[i].getY()
            );
            mInverseKinematics.setRow(i * 2 + 1, 0, 
                0, 1, +mModules[i].getX()
            );

            mInverseKinematicsPrime.setRow(i * 2 + 0, 0,
                1, 0, -mModules[i].getX(), -mModules[i].getY() 
            );
            mInverseKinematicsPrime.setRow(i * 2 + 1, 0,
                0, 1, -mModules[i].getY(), +mModules[i].getX()
            );
        }

        mForwardKinematics = mInverseKinematics.pseudoInverse();
        mForwardKinematicsPrime = mInverseKinematicsPrime.pseudoInverse();
    }

    /**
     * Converts robot relative Chassis Speeds into Swerve Module States
     * @param speeds Desired speeds of the robot
     * @param centerOfRotation Point at which the robot will rotate around
     * @return
     */
    public SO_SwerveModuleState[] toSwerveModuleStates(SO_ChassisSpeeds speeds, Translation2d centerOfRotation) {
        // Returns empty module states if speeds are zero
        if (speeds.vxMetersPerSecond == 0.0
            && speeds.vyMetersPerSecond == 0.0
            && speeds.omegaRadiansPerSecond == 0.0) {
            SO_SwerveModuleState[] newStates = new SO_SwerveModuleState[mModuleNum];
            for (int i = 0; i < mModuleNum; i++) {
                newStates[i] = new SO_SwerveModuleState(0.0, 0.0, mModuleStates[i].angle, Rotation2d.fromDegrees(0));
            }

            mModuleStates = newStates;
            return mModuleStates;
        }

        // Limit computing robot reference vectors until center of rotation changes
        if (!mPreviousCenterOfRotation.equals(centerOfRotation)) {
            for (int i = 0; i < mModuleNum; i++) {
                mInverseKinematics.setRow(i * 2 + 0, 0, 
                    1, 0, -mModules[i].getY() + centerOfRotation.getY()
                );
                mInverseKinematics.setRow(i * 2 + 1, 0, 
                    0, 1, +mModules[i].getX() - centerOfRotation.getX()
                );
    
                mInverseKinematicsPrime.setRow(i * 2 + 0, 0,
                    1, 0, -mModules[i].getX() + centerOfRotation.getX(), -mModules[i].getY() + centerOfRotation.getY() 
                );
                mInverseKinematicsPrime.setRow(i * 2 + 1, 0,
                    0, 1, -mModules[i].getY() + centerOfRotation.getY(), +mModules[i].getX() - centerOfRotation.getX()
                );
            }
            mPreviousCenterOfRotation = centerOfRotation;
        }

        // First order ChassisSpeeds
        SimpleMatrix chassisSpeedsMatrix_vPlusOmega = new SimpleMatrix(3, 1);
        chassisSpeedsMatrix_vPlusOmega.setColumn(0, 0,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
        // Second order ChassisSpeeds
        SimpleMatrix chassisAccelMatrix_aPlusAlpha = new SimpleMatrix(4, 1);
        chassisAccelMatrix_aPlusAlpha.setColumn(0, 0,
            speeds.axMetersPerSecondSquared,
            speeds.ayMetersPerSecondSquared,
            Math.pow(speeds.omegaRadiansPerSecond, 2),
            speeds.alphaRadiansPerSecondSquared
        );

        // First order kinematics
        // Sovles for Velocity and Heading
        SimpleMatrix moduleStateMatrix_vm = mInverseKinematics.mult(chassisSpeedsMatrix_vPlusOmega);
        SimpleMatrix moduleStateMatrix_am = mInverseKinematicsPrime.mult(chassisAccelMatrix_aPlusAlpha);
        
        mModuleStates = new SO_SwerveModuleState[mModuleNum];

        for (int i = 0; i < mModuleNum; i++) {
            // Gets Velocity and Acceleration components from matrices 
            double vx = moduleStateMatrix_vm.get(i * 2 + 0, 0);
            double vy = moduleStateMatrix_vm.get(i * 2 + 1, 0);
            double ax = moduleStateMatrix_am.get(i * 2 + 0, 0);
            double ay = moduleStateMatrix_am.get(i * 2 + 1, 0);
            // System.out.println("x " + ax +", y " + ay);

            // Converts Velocity components into Angle heading and Linear Velocity
            double velocity = Math.hypot(vx, vy);
            Rotation2d heading = new Rotation2d(vx, vy);
            // System.out.println(heading);

            // Converts Acceleration components into Angular velocity and Liner Acceleration
            SimpleMatrix headingMatrix = new SimpleMatrix(2, 2);
            headingMatrix.setRow(0, 0, 
                +heading.getCos(), +heading.getSin()
            );
            headingMatrix.setRow(1, 0, 
                -heading.getSin(), +heading.getCos()
            );
            SimpleMatrix componentMatrix = new SimpleMatrix(2, 1);
            componentMatrix.setColumn(0, 0, 
                ax, 
                ay
            );

            SimpleMatrix moduleAccelerationMatrix = headingMatrix.mult(componentMatrix);

            double accel = moduleAccelerationMatrix.get(0, 0);
            Rotation2d angularVelocity = Rotation2d.fromRadians(moduleAccelerationMatrix.get(1, 0)).div(velocity);
            // Takes care of when velocity is 0 and angular velocity becomes -Infinity
            if (velocity == 0) angularVelocity = Rotation2d.fromDegrees(0.0); // Prolly not mathematically correct but idk
            // Passes calculated values into module states
            mModuleStates[i] = new SO_SwerveModuleState(
                velocity, accel, 
                heading, angularVelocity
            );
        }

        return mModuleStates;
    }

    public SO_SwerveModuleState[] toSwerveModuleStates(SO_ChassisSpeeds speeds) {
        return toSwerveModuleStates(speeds, new Translation2d());
    }

    /**
     * Converts Swerve Module States into Chassis Speeds
     * @param moduleStates Module Speeds to convert 
     * @return Equivilent Chassis Speeds
     */
    public SO_ChassisSpeeds toChassisSpeeds(SO_SwerveModuleState... moduleStates) { 
        if (moduleStates.length != mModuleNum) {
            throw new IllegalArgumentException("Number of module states inconsistant with constructor");
        }

        SimpleMatrix moduleStateVelMatrix = new SimpleMatrix(mModuleNum * 2, 1);
        SimpleMatrix moduleStateAccelMatrix = new SimpleMatrix(mModuleNum * 2, 1);

        for (int i = 0; i < mModuleNum; i++) {
            SO_SwerveModuleState module = moduleStates[i];
            // Easily grab velocity components of the module
            moduleStateVelMatrix.set(i * 2 + 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStateVelMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());

            // Matrix multiplication for acceleration components of the module 
            SimpleMatrix headingMatrix = new SimpleMatrix(2, 2);
            headingMatrix.setRow(0, 0,
                +module.angle.getCos(), -module.angle.getSin()
            );
            headingMatrix.setRow(1, 0, 
                +module.angle.getSin(), +module.angle.getCos()
            );
            SimpleMatrix linearAccelMatrix = new SimpleMatrix(2, 1);
            linearAccelMatrix.setColumn(0, 0, 
                module.accelMetersPerSecondSquared, 
                module.speedMetersPerSecond * module.angleVel.getRadians()
            );
            SimpleMatrix componentAccelMatrix = headingMatrix.mult(linearAccelMatrix);

            moduleStateAccelMatrix.set(i * 2 + 0, componentAccelMatrix.get(0, 0));
            moduleStateAccelMatrix.set(i * 2 + 1, componentAccelMatrix.get(1, 0));
        }

        // Inverses module components to chassis relative components
        SimpleMatrix chassisSpeedsVector = mForwardKinematics.mult(moduleStateVelMatrix);
        SimpleMatrix chassisAccelVector = mForwardKinematicsPrime.mult(moduleStateAccelMatrix);

        return new SO_ChassisSpeeds(
            chassisSpeedsVector.get(0, 0), 
            chassisSpeedsVector.get(1, 0), 
            chassisSpeedsVector.get(2, 0), 
            chassisAccelVector.get(0, 0),
            chassisAccelVector.get(1, 0),
            chassisAccelVector.get(3, 0)
        );
    }
}
