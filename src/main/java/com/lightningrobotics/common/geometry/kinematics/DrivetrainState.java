package com.lightningrobotics.common.geometry.kinematics;

/**
 * Base class for a drivetrain state, such a collection of 
 * {@link com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState Module States}.
 */
public abstract class DrivetrainState {

    public abstract DrivetrainState getState();

}

