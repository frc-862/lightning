package com.lightningrobotics.common.geometry.kinematics;

/**
 * Base kinematics interface.
 * Implementing classes will implement forward and inverse kinematics
 * calculations, as well as speed normalization.
 */
public interface LightningKinematics {

    public DrivetrainSpeed forward(DrivetrainState state);

    public DrivetrainState inverse(DrivetrainSpeed speed);

}
