package com.lightningrobotics.common.geometry;

import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;

/**
 * Base kinematics interface. Implementing classes will provide 2 functions:
 * 
 * <p>1. Ability to convert to {@link com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed} object.
 * 
 * <p>2. Ability to constrain speed to realistic value for robot.
 */
public interface LightningKinematics {

    public void normalize(double maxSpeed);

    public DrivetrainSpeed toDrivetrainSpeed();
    
}
